
import io
import mmap
import time
from datetime import datetime

import pandas as pd

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rosfl_interfaces.srv import NodeRegistration, ModelTransmission, ModelTrian, TestMetrics, TrainMetrics
from rosfl_interfaces.msg import ModelBlob


import copy
import torch
import torch.nn as nn
import numpy as np
import os
from torch.utils.data import DataLoader
from sklearn.preprocessing import label_binarize
from sklearn import metrics
from rosfl.utils.rosconfig_utils import configure_algorithm, configure_model, load_rosconfig
from rosfl.utils.data_utils import read_client_data


class Client(Node):
    """
    Base class for clients in federated learning.
    """
    def __init__(self, package_name, node_name='ros_client', node_config='FedAvg', condition=None, **kwargs):

        super().__init__(node_name)
        self.node_name = node_name
        self.package_name = package_name
        self.share_dir = get_package_share_directory(self.package_name)
        client_config = os.path.join(self.share_dir, 'datas/config', node_config, 'client.yaml')
        args = load_rosconfig(client_config)
        self.model_name = args.model
        args.model = configure_model(args)
        args = configure_algorithm(args)
        self.args = args

        self.init_FL_config()

        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100)


        self.declare_parameter('id', -1)
        self.declare_parameter('train_samples', -1)
        self.declare_parameter('test_samples', -1)
        self.declare_parameter('train_slow', False)
        self.declare_parameter('send_slow', False)
        self.id = self.get_parameter('id').value
        self.train_samples = self.get_parameter('train_samples').value
        self.test_samples = self.get_parameter('test_samples').value
        self.train_slow = self.get_parameter('train_slow').value
        self.send_slow = self.get_parameter('send_slow').value


        self.add_on_set_parameters_callback(self.update_client_basic_parameters)

        self.srv_receive_model = None
        self.srv_start_train = None
        self.srv_test_metrics = None
        self.srv_train_metrics = None

        self.sub_model = None

        self.client_cb_group = ReentrantCallbackGroup()
        self.server_cb_group = ReentrantCallbackGroup()

        self.init_services()
        self.init_topics()

        self.get_logger().info(f"node->{self.node_name} basic initialization completed")

    def init_FL_config(self):

        torch.manual_seed(0)
        self.model = copy.deepcopy(self.args.model)
        self.algorithm = self.args.algorithm
        self.dataset = self.args.dataset
        self.device = self.args.device
        self.save_folder_name = self.args.save_folder_name
        self.num_classes = self.args.num_classes

        self.batch_size = self.args.batch_size
        self.learning_rate = self.args.local_learning_rate
        self.local_epochs = self.args.local_epochs
        self.few_shot = self.args.few_shot


        self.has_BatchNorm = False
        for layer in self.model.children():
            if isinstance(layer, nn.BatchNorm2d):
                self.has_BatchNorm = True
                break

        self.train_time_cost = {'num_rounds': 0, 'total_cost': 0.0}
        self.send_time_cost = {'num_rounds': 0, 'total_cost': 0.0}


        self.loss = nn.CrossEntropyLoss()

        self.optimizer = torch.optim.SGD(self.model.parameters(), lr=self.learning_rate)

        self.learning_rate_scheduler = torch.optim.lr_scheduler.ExponentialLR(
            optimizer=self.optimizer,
            gamma=self.args.learning_rate_decay_gamma
        )
        self.learning_rate_decay = self.args.learning_rate_decay

        self.model_version = 0
        self.rs_test_local_acc_trained  = [0.0]
        self.rs_test_local_auc_trained  = [0.0]
        self.rs_test_global_acc_trained = [0.0]
        self.rs_test_global_auc_trained = [0.0]
        self.rs_train_loss_trained = [0.0]

        self.rs_test_local_acc_aggregated  = []
        self.rs_test_local_auc_aggregated  = []
        self.rs_test_global_acc_aggregated = []
        self.rs_test_global_auc_aggregated = []
        self.rs_train_loss_aggregated = []

    def init_services(self):
        self.srv_receive_model = self.create_service(
            ModelTransmission,
            self.node_name + '/receive_model',
            self.receive_model_from_server,
            callback_group=self.server_cb_group
        )
        self.srv_start_train = self.create_service(
            ModelTrian,
            self.node_name + '/start_train',
            self.start_model_train,
            callback_group=self.server_cb_group
        )
        self.srv_test_metrics = self.create_service(
            TestMetrics,
            self.node_name + '/test_metrics',
            self.start_test_metrics_aggregated,
            callback_group=self.server_cb_group
        )
        self.srv_train_metrics = self.create_service(
            TrainMetrics,
            self.node_name + '/train_metrics',
            self.start_train_metrics_aggregated,
            callback_group=self.server_cb_group
        )

    def init_topics(self):
        pass

    def update_client_basic_parameters(self, parameters):
        for parameter in parameters:

            if parameter.name == 'id':
                self.id = parameter.value
            if parameter.name == 'train_samples':
                self.train_samples = parameter.value
            if parameter.name == 'test_samples':
                self.test_samples = parameter.value
            if parameter.name == 'train_slow':
                self.train_slow = parameter.value
            if parameter.name == 'send_slow':
                self.send_slow = parameter.value
        return SetParametersResult(successful=True)

    def request_register_to_server(self):
        node_name = self.node_name
        service_api = '/'+self.args.target_server+'/register_node'
        register_request = self.create_client(NodeRegistration, service_api, callback_group=self.client_cb_group)
        while register_request.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().warning(f"Waiting for {self.args.target_server} registration service...")
        self.get_logger().info(f"{self.node_name} starts registration")
        request = NodeRegistration.Request()
        request.node_name = node_name
        future = register_request.call_async(request)
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=1.0)
        self.handle_register(future)
        register_request.destroy()

    def handle_register(self, future):
        response = future.result()
        self.set_parameters([rclpy.Parameter('id', rclpy.Parameter.Type.INTEGER, response.node_id),
                             rclpy.Parameter('train_samples', rclpy.Parameter.Type.INTEGER, response.train_samples),
                             rclpy.Parameter('test_samples', rclpy.Parameter.Type.INTEGER, response.test_samples),
                             rclpy.Parameter('train_slow', rclpy.Parameter.Type.BOOL, response.train_slow),
                             rclpy.Parameter('send_slow', rclpy.Parameter.Type.BOOL, response.send_slow)
                             ])
        self.get_logger().info(f"{self.node_name} registration completed")

    def receive_model_from_server(self, request, response):

        try:
            response.node_name = self.node_name
            buffer = None
            buffer = io.BytesIO(request.model_data)
            state_dict = torch.load(buffer)
            tmp_model = copy.deepcopy(self.model)
            tmp_model.load_state_dict(state_dict, strict=True)
            self.set_model_parameters(tmp_model)
            response.success = True
            self.get_logger().info(f"receive_model->{self.node_name} model updated")
        except Exception as e:
            self.get_logger().warning(e)
            response.success = False
        return response

    def start_model_train(self, request, response):
        response.node_id = request.id
        upload_success = request.upload_success
        train_success = request.train_success
        finish_rate = round(min(float(request.finish_rate), 1.0), 3)
        try:
            assert request.id == self.id and request.node_name == self.node_name
            self.get_logger().info(f"Client {self.id} received training request")
            if not train_success:
                assert train_success, f"Model training failed: {train_success}"
            global_state_dict = {k: v.clone() for k, v in self.model.state_dict().items()}

            self.train()
            local_state_dict = self.model.state_dict()
            trainable_keys = [name for name, param in self.model.named_parameters() if param.requires_grad]
            total_trainable_params = 0
            total_trainable_bytes = 0
            for k in trainable_keys:
                tensor = local_state_dict[k]
                total_trainable_params += tensor.numel()
                total_trainable_bytes += tensor.numel() * tensor.element_size()
            head_keys = [k for k in trainable_keys if 'head' in k]
            base_keys = [k for k in trainable_keys if 'head' not in k]

            head_elements = sum(local_state_dict[k].numel() for k in head_keys)
            base_elements = sum(local_state_dict[k].numel() for k in base_keys)
            total_trainable_elements = head_elements + base_elements

            elements_quota = int(total_trainable_elements * finish_rate)


            head_keep_rate = finish_rate
            base_keep_rate = finish_rate

            elements_kept_head = 0
            elements_kept_base = 0


            for key in head_keys:
                local_tensor = local_state_dict[key]
                global_tensor = global_state_dict[key]
                delta_w = local_tensor - global_tensor

                if head_keep_rate >= 0.999:
                    elements_kept_head += local_tensor.numel()
                    continue

                prob_tensor = torch.full_like(delta_w, float(head_keep_rate), dtype=torch.float32)
                mask = torch.bernoulli(prob_tensor).to(delta_w.dtype)
                local_state_dict[key] = global_tensor + (delta_w * mask)
                elements_kept_head += int(mask.sum().item())


            for key in base_keys:
                local_tensor = local_state_dict[key]
                global_tensor = global_state_dict[key]
                delta_w = local_tensor - global_tensor

                if base_keep_rate >= 0.999:
                    elements_kept_base += local_tensor.numel()
                    continue
                elif base_keep_rate <= 0.001:
                    local_state_dict[key] = global_tensor.clone()
                    continue

                prob_tensor = torch.full_like(delta_w, float(base_keep_rate), dtype=torch.float32)
                mask = torch.bernoulli(prob_tensor).to(delta_w.dtype)
                local_state_dict[key] = global_tensor + (delta_w * mask)
                elements_kept_base += int(mask.sum().item())

            buffer = io.BytesIO()
            torch.save(local_state_dict, buffer)
            data_bytes = buffer.getvalue()
            data_size = len(data_bytes)

            response.model_data = data_bytes

            response.success = True
            response.model_size = data_size
            response.num_rounds = self.train_time_cost['num_rounds']
            response.train_samples = self.train_samples
            response.total_cost = self.train_time_cost['total_cost']
            self.get_logger().info(f"Client {self.id} completed this round")

        except Exception as e:
            self.get_logger().warning(f"Training or transmission exception: {e}")
            response.success = False
            response.model_size = 0
            response.num_rounds = -1
            response.train_samples = -1
            response.total_cost = -1.0

        return response

    def train(self):
        self.get_logger().info(f"{self.node_name} needs to implement train()")

    def set_model_parameters(self, model):
        for new_param, old_param in zip(model.parameters(), self.model.parameters()):
            old_param.data = new_param.data.clone()

    def clone_model_model(self, model, target):
        for param, target_param in zip(model.parameters(), target.parameters()):
            target_param.data = param.data.clone()

    def update_model_parameters(self, model, new_params):
        for param, new_param in zip(model.parameters(), new_params):
            param.data = new_param.data.clone()

    def show_model_parameters(self, model):
        for param in zip(model.parameters()):
            print(param)
        print("---------------------------------------------------")

    def load_train_data(self, batch_size=None):
        if batch_size is None:
            batch_size = self.batch_size
        train_data = read_client_data(self.dataset, self.share_dir, self.id, is_train=True, few_shot=self.few_shot)
        return DataLoader(train_data, batch_size, drop_last=True, shuffle=True)

    def load_local_test_data(self, batch_size=None):
        if batch_size is None:
            batch_size = self.batch_size
        test_data = read_client_data(self.dataset, self.share_dir, self.id, is_train=False, few_shot=self.few_shot)
        return DataLoader(test_data, batch_size, drop_last=False, shuffle=True)

    def load_global_test_data(self, batch_size=None):
        if batch_size is None:
            batch_size = self.batch_size
        test_data = read_client_data(self.dataset, self.share_dir, "global", is_train=False, few_shot=self.few_shot)
        return DataLoader(test_data, batch_size, drop_last=False, shuffle=True)

    def test_metrics(self, testloaderfull):
        self.get_logger().info("Starting test evaluation (base)")

        self.model.eval()
        test_acc = 0
        test_num = 0
        y_prob = []
        y_true = []

        with torch.no_grad():
            for x, y in testloaderfull:
                if isinstance(x, list):
                    x[0] = x[0].to(self.device)
                else:
                    x = x.to(self.device)
                y = y.to(self.device)
                output = self.model(x)

                test_acc += (torch.sum(torch.argmax(output, dim=1) == y)).item()
                test_num += y.shape[0]

                y_prob.append(output.detach().cpu().numpy())
                nc = self.num_classes
                if self.num_classes == 2:
                    nc += 1
                lb = label_binarize(y.detach().cpu().numpy(), classes=np.arange(nc))
                if self.num_classes == 2:
                    lb = lb[:, :2]
                y_true.append(lb)

        y_prob = np.concatenate(y_prob, axis=0)
        y_true = np.concatenate(y_true, axis=0)

        auc = metrics.roc_auc_score(y_true, y_prob, average='micro')
        return test_acc, test_num, auc

    def test_local_metrics(self):
        data = self.load_local_test_data()
        test_local_acc, test_local_num, local_auc = self.test_metrics(data)
        return test_local_acc, test_local_num, local_auc

    def test_global_metrics(self):
        data = self.load_global_test_data()
        test_global_acc, test_global_num, global_auc = self.test_metrics(data)
        return test_global_acc, test_global_num, global_auc

    def test_metrics_trained(self):

        test_local_acc, test_local_num, local_auc = self.test_local_metrics()
        test_local_acc_trained = test_local_acc * 1.0 / test_local_num
        test_local_auc_trained = local_auc * 1.0

        test_global_acc, test_global_num, global_auc = self.test_global_metrics()
        test_global_acc_trained = test_global_acc * 1.0 / test_global_num
        test_global_auc_trained = global_auc * 1.0

        self.rs_test_local_acc_trained.append(test_local_acc_trained)
        self.rs_test_local_auc_trained.append(test_local_auc_trained)
        self.rs_test_global_acc_trained.append(test_global_acc_trained)
        self.rs_test_global_auc_trained.append(test_global_auc_trained)

        self.get_logger().info(f"Post-training test evaluation completed (base): test_local_acc->{test_local_acc_trained}, test_local_auc->{test_local_auc_trained}, test_global_acc->{test_global_acc_trained}, test_global_auc->{test_global_auc_trained}")

    def test_metrics_aggregated(self):
        test_local_acc, test_local_num, local_auc = self.test_local_metrics()
        test_local_acc_aggregated = test_local_acc * 1.0 / test_local_num
        test_local_auc_aggregated = local_auc * 1.0

        test_global_acc, test_global_num, global_auc = self.test_global_metrics()
        test_global_acc_aggregated = test_global_acc * 1.0 / test_global_num
        test_global_auc_aggregated = global_auc * 1.0

        self.rs_test_local_acc_aggregated.append(test_local_acc_aggregated)
        self.rs_test_local_auc_aggregated.append(test_local_auc_aggregated)
        self.rs_test_global_acc_aggregated.append(test_global_acc_aggregated)
        self.rs_test_global_auc_aggregated.append(test_global_auc_aggregated)

        self.get_logger().info(f"Post-aggregation test evaluation completed (base): test_local_acc->{test_local_acc_aggregated}, test_local_auc->{test_local_auc_aggregated}, test_global_acc->{test_global_acc_aggregated}, test_global_auc->{test_global_auc_aggregated}")

        return test_local_acc, test_local_num, local_auc


    def start_test_metrics_aggregated(self, request, response):
        response.node_id = request.id
        try:
            assert request.id == self.id and request.node_name == self.node_name
            test_acc, test_num, auc = self.test_metrics_aggregated()
            response.success = True
            response.test_acc = float(test_acc)
            response.test_num = test_num
            response.auc = float(auc)
        except Exception as e:
            self.get_logger().warning(f"{e}")
            response.success = False
            response.test_acc = -1.0
            response.test_num = 0
            response.auc = -1.0
        return response

    def train_metrics(self):
        self.get_logger().info("Starting train evaluation (base)")
        train_loader = self.load_train_data()
        self.model.eval()
        train_num = 0
        losses = 0

        with torch.no_grad():
            for x, y in train_loader:
                if isinstance(x, list):
                    x[0] = x[0].to(self.device)
                else:
                    x = x.to(self.device)
                y = y.to(self.device)
                output = self.model(x)
                loss = self.loss(output, y)
                train_num += y.shape[0]
                losses += loss.item() * y.shape[0]
        return losses, train_num

    def train_metrics_trained(self):
        losses_trained, train_num_trained = self.train_metrics()
        train_loss_trained = losses_trained * 1.0 / train_num_trained
        self.rs_train_loss_trained.append(train_loss_trained)
        self.get_logger().info(f"Post-training loss evaluation completed (base): loss->{train_loss_trained}")
        pass

    def train_metrics_aggregated(self):
        losses_aggregated, train_num_aggregated = self.train_metrics()
        train_loss_aggregated = losses_aggregated * 1.0 / train_num_aggregated
        self.rs_train_loss_aggregated.append(train_loss_aggregated)
        self.get_logger().info(f"Post-aggregation loss evaluation completed (base): loss->{train_loss_aggregated}")
        return losses_aggregated, train_num_aggregated

    def start_train_metrics_aggregated(self, request, response):
        response.node_id = request.id
        try:
            assert request.id == self.id and request.node_name == self.node_name
            losses, train_num = self.train_metrics_aggregated()
            response.success = True
            response.losses = float(losses)
            response.train_num = int(train_num)
        except Exception as e:
            self.get_logger().warning(f"{e}")
            response.success = False
            response.losses = -1.0
            response.train_num = 0

        return response

    def _safe_last(self, seq):
        return seq[-1] if seq else None