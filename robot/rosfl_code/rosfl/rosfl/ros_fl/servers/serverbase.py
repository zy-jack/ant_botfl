import shutil
import json
from collections import OrderedDict
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosfl_interfaces.srv import NodeRegistration, ModelTransmission, ModelTrian, TestMetrics, TrainMetrics, CmdAction
from rosfl_interfaces.msg import ModelBlob


import torch
import os
import io
import pandas as pd
import numpy as np
import copy
import time
from torch.utils.data import DataLoader
from sklearn.preprocessing import label_binarize
from sklearn import metrics


from rosfl.utils.rosconfig_utils import configure_algorithm, configure_model, load_rosconfig
from rosfl.utils.data_utils import read_client_data


MODE_CONTROLL = 'CONTROLL'

class Server(Node):
    def __init__(self, package_name, node_name='ros_server', node_config='FedAvg', condition=None, **kwargs):

        times = 0
        super().__init__(node_name)
        self.package_name = package_name
        self.node_name = node_name
        self.share_dir = get_package_share_directory(self.package_name)
        self.server_config = os.path.join(self.share_dir, 'datas/config', node_config, 'server.yaml')
        self.client_config = os.path.join(self.share_dir, 'datas/config', node_config, 'client.yaml')
        args = load_rosconfig(self.server_config)
        self.model_name = args.model
        args.model = configure_model(args)
        args = configure_algorithm(args)
        self.args = args

        self.init_fl_config()

        buffer = io.BytesIO()
        torch.save(self.global_model.state_dict(), buffer)
        data_bytes = buffer.getvalue()
        data_size = len(data_bytes)

        self.registered_clients = []
        self.online_states = []
        self.online_clients_id = []
        self.registered_num = 0
        self.online_num = 0
        self.clients_parameters = []


        self.clients_ = []

        self.times = times

        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100)

        self.upload_clients = {}


        self.set_slow_clients(self.num_clients)
        self.generate_clients_parameters(self.num_clients)


        self.srv_execute_action = None
        self.srv_register_node = None
        self.pub_model = None

        self.client_cb_group = ReentrantCallbackGroup()
        self.server_cb_group = ReentrantCallbackGroup()
        self.server_execute_action = ReentrantCallbackGroup()

        self.init_services()
        self.init_topics()

        self.get_logger().info(f"node->{self.node_name} basic initialization completed")

    def init_fl_config(self):
        self.device = self.args.device
        self.dataset = self.args.dataset
        self.num_classes = self.args.num_classes
        self.global_rounds = self.args.global_rounds
        self.local_epochs = self.args.local_epochs
        self.batch_size = self.args.batch_size
        self.learning_rate = self.args.local_learning_rate
        self.global_model = copy.deepcopy(self.args.model)
        self.num_clients = self.args.num_clients
        self.join_ratio = self.args.join_ratio
        self.random_join_ratio = self.args.random_join_ratio
        self.num_join_clients = 0
        self.current_num_join_clients = 0
        self.few_shot = self.args.few_shot
        self.algorithm = self.args.algorithm
        self.time_select = self.args.time_select
        self.goal = self.args.goal
        self.save_folder_name = self.args.save_folder_name
        self.top_cnt = self.args.top_cnt
        self.auto_break = self.args.auto_break
        self.timeout_check = self.args.timeout_check
        self.timeout_train_model = self.args.timeout_train_model
        self.timeout_send_model = self.args.timeout_send_model
        self.timeout_metrics = self.args.timeout_metrics
        self.time_error = self.args.time_error


        self.selected_clients = []
        self.selected_nums = 0
        self.selected_online_nums = 0

        self.train_slow_clients = []
        self.send_slow_clients = []

        self.model_version = 0
        self.send_model_ids = []
        self.receive_model_ids = []
        self.uploaded_weights = []
        self.uploaded_ids = []
        self.uploaded_models = []

        self.rs_test_micro_acc_local = []
        self.rs_test_micro_auc_local = []
        self.rs_test_macro_acc_local = []
        self.rs_test_macro_acc_std_local = []
        self.rs_test_macro_acc_worst_local = []
        self.rs_test_macro_auc_local = []
        self.rs_test_macro_auc_std_local = []
        self.rs_test_macro_auc_worst_local = []
        self.rs_train_loss_local = []

        self.rs_test_acc_global = []
        self.rs_test_auc_global = []

        self.Budget = []

        self.eval_gap = self.args.eval_gap
        self.client_drop_rate = self.args.client_drop_rate
        self.train_slow_rate = self.args.train_slow_rate
        self.send_slow_rate = self.args.send_slow_rate


        self.update_config_futures = {}
        self.send_model_futures = {}
        self.train_model_results = {}
        self.train_futures = {}
        self.test_metrics_results = {}
        self.test_metrics_futures = {}
        self.train_metrics_results = {}
        self.train_metrics_futures = {}

    def init_services(self):
        self.srv_register_node = self.create_service(
            NodeRegistration,
            self.node_name + '/register_node',
            self.request_register_from_client,
            callback_group=self.server_cb_group
        )
        self.srv_execute_action = self.create_service(
            CmdAction,
            self.node_name + '/execute_action',
            self.execute_action_command,
            callback_group=self.server_execute_action
        )

    def init_topics(self):
        pass

    def execute_action_command(self, request, response):
        mode = request.mode
        try:
            self.get_logger().info(f"mode->{mode}")
            match mode:
                case 'CONTROLL':
                    selected_clients = request.param
                    json_upload_data = request.json_param
                    self.upload_clients = {int(k): float(v) for k, v in json.loads(json_upload_data).items()}
                    if len(selected_clients) > 0:
                        self.get_logger().info(f"nodes->{self.upload_clients}")
                        self.train()
                    else:
                        assert False, f"mode->{mode}, param is invalid"
                case 'PING':
                    self.get_logger().info(f"PING FINISHED")
                case _:
                    assert False, f"mode->{mode} is undefined"
            response.success = True
        except Exception as e:
            self.get_logger().warning(str(e))
            response.success = False
        return response

    def request_register_from_client(self, request, response):
        node_name = request.node_name


        self.get_logger().info(f"node->{node_name} is registering")
        response.node_id = self.registered_num
        self.registered_num = self.registered_num + 1
        self.registered_clients.append(node_name)
        self.online_states.append(True)
        parameters = self.clients_parameters[response.node_id]
        response.train_samples = parameters['train_samples']
        response.test_samples = parameters['test_samples']
        response.train_slow = parameters['train_slow']
        response.send_slow = parameters['send_slow']
        return response


    def generate_clients_parameters(self, num_clients):

        self.clients_parameters = []
        for i, train_slow, send_slow in zip(range(num_clients), self.train_slow_clients, self.send_slow_clients):
            train_data = read_client_data(self.dataset, self.share_dir, i, is_train=True, few_shot=self.few_shot)
            test_data = read_client_data(self.dataset, self.share_dir, i, is_train=False, few_shot=self.few_shot)
            parameters = dict()
            parameters['train_samples'] = len(train_data)
            parameters['test_samples'] = len(test_data)
            parameters['train_slow'] = train_slow
            parameters['send_slow'] = send_slow
            self.clients_parameters.append(parameters)

    def get_client_id(self, target_client):
        id = self.registered_clients.index(target_client)
        if id < 0:
            self.get_logger().warning(f"{target_client} has not registered with {self.node_name}")
            return None
        else:
            return id

    def check_online(self):
        if self.registered_num == 0:
            self.get_logger().warning(
                datetime.now().strftime("%Y-%m-%d %H:%M:%S") + f": no clients have registered with {self.node_name}")
            self.online_num = 0
        else:

            selected_online_nums = 0
            self.selected_clients = self.selected_clients if self.selected_clients != [] else list(range(0, self.registered_num, 1))
            for client_id in self.selected_clients:
                self.check_state(client_id)
                if self.online_states[client_id]:
                    selected_online_nums += 1
            self.selected_online_nums = selected_online_nums
            self.num_join_clients = int(self.selected_online_nums * self.join_ratio)


            self.online_clients_id = []
            for index in range(self.registered_num):
                if self.online_states[index]:
                    self.online_clients_id.append(index)
            self.online_num = len(self.online_clients_id)

        self.get_logger().info(f"check online->{self.node_name} online clients: {self.online_num}")

    def check_state(self, client_id):
        client_name = self.registered_clients[client_id]
        client_api = '/'+client_name+'/start_train'
        start_time = time.time()

        cli = self.create_client(ModelTrian, client_api, callback_group=self.client_cb_group)
        is_online = True
        while not cli.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > self.timeout_check:
                self.get_logger().warning(f"Waiting for client {client_id} to go offline")
                is_online = False
                break
        self.online_states[client_id] = is_online
        self.destroy_client(cli)


    def select_slow_clients(self, slow_rate, num_clients):
        slow_clients = [False for i in range(num_clients)]
        idx = [i for i in range(num_clients)]
        idx_ = np.random.choice(idx, int(slow_rate * num_clients))
        for i in idx_:
            slow_clients[i] = True
        return slow_clients


    def set_slow_clients(self, num_clients):
        self.train_slow_clients = self.select_slow_clients(self.train_slow_rate, num_clients)
        self.send_slow_clients = self.select_slow_clients(self.send_slow_rate, num_clients)


    def select_clients(self, selected_clients = []):
        self.selected_clients = selected_clients
        self.check_online()
        assert self.online_num > 0, f"Error: {self.node_name} online clients: {self.online_num}"

        if self.random_join_ratio:
            self.current_num_join_clients =\
            np.random.choice(range(self.num_join_clients, self.selected_online_nums + 1), 1, replace=False)[0]
        else:
            self.current_num_join_clients = self.num_join_clients

        selected_clients = list(np.random.choice(self.online_clients_id, self.current_num_join_clients, replace=False))
        self.selected_nums = len(selected_clients)
        return selected_clients

    def send_model(self, client_name, data_bytes):
        client_api = '/'+client_name+'/receive_model'
        start_time = time.time()
        send_model_to_client = self.create_client(ModelTransmission, client_api, callback_group=self.client_cb_group)
        while not send_model_to_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Waiting for client {client_name} receive-model service...")
            if time.time() - start_time > self.timeout_check:
                self.get_logger().warning(f"Timeout waiting for node {client_name} service response; continuing")
                return
        req = ModelTransmission.Request()
        req.model_size = len(data_bytes)
        req.model_data = data_bytes
        future = send_model_to_client.call_async(req)
        self.send_model_futures[client_name] = (future, send_model_to_client)

    def send_models(self, version=0):
        self.check_online()
        self.send_model_ids = []
        self.send_model_futures = {}
        assert self.online_num > 0, f"Error: {self.node_name} online clients: {self.online_num}"
        buffer = io.BytesIO()
        torch.save(self.global_model.state_dict(), buffer)
        data_bytes = buffer.getvalue()
        data_size = len(data_bytes)
        for index in self.online_clients_id:
            client_name = self.registered_clients[index]
            self.send_model(client_name, data_bytes)
        self.get_logger().info("Waiting for all clients to update model results...")

        start_time = time.time()
        while rclpy.ok() and self.online_num!=0 and any(not fut.done() for fut, _ in self.send_model_futures.values()):
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - start_time > self.timeout_send_model:
                self.get_logger().warning("Timeout waiting for all clients to update model responses; continuing")
                break
        for client_name, (future, cli) in self.send_model_futures.items():
            try:
                assert future.done()
                assert future.result().success
                client_id = self.get_client_id(client_name)
                self.send_model_ids.append(client_id)
                self.get_logger().info(f"Received feedback from {client_name}")
            except Exception as e:
                print(future.done())
                print(future.result().success)
                self.get_logger().warning(f"Error processing result from client {client_name}: {e}")
                cli.remove_pending_request(future)
                future.cancel()
            finally:
                self.destroy_client(cli)
        self.get_logger().info(f"All clients completed model updates, online clients: {self.online_num}")

    def request_start_train(self, client_id):
        client_name = self.registered_clients[client_id]
        client_api = '/'+client_name+'/start_train'
        start_time = time.time()
        request_start_train_to_client = self.create_client(ModelTrian, client_api, callback_group=self.client_cb_group)
        while not request_start_train_to_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Waiting for client {client_id} train-evaluation service...")
            if time.time() - start_time > self.timeout_check:
                self.get_logger().warning(f"Timeout waiting for client {client_id} service response; continuing")
                return
        req = ModelTrian.Request()
        req.id = int(client_id)
        req.node_name = client_name
        train_finished = True
        upload_finished = True
        finish_rate = self.upload_clients[client_id]
        req.upload_success = upload_finished
        req.train_success = train_finished
        req.finish_rate = finish_rate
        future = request_start_train_to_client.call_async(req)
        self.train_futures[client_id] = (future, request_start_train_to_client)
        self.get_logger().info(f"Training request sent to client {client_id}")

    def handle_train_model_result(self, future):
        train_res = {}
        response = future.result()
        client_id = response.node_id
        client_name = self.registered_clients[client_id]
        self.receive_model_ids.append(client_id)
        model = copy.deepcopy(self.global_model)
        buffer = None
        buffer = io.BytesIO(response.model_data)
        state_dict = torch.load(buffer)
        model.load_state_dict(state_dict)
        train_res['model'] = model
        train_res['num_rounds'] = response.num_rounds
        train_res['train_samples'] = response.train_samples
        train_res['total_cost'] = response.total_cost
        self.train_model_results[client_id] = train_res

    def start_train(self, round = 0):
        self.check_online()
        assert self.selected_online_nums > 0, f"Error: {self.node_name} selected online clients: {self.online_num}"
        self.train_model_results = {}
        self.train_futures = OrderedDict()

        for client_id in self.selected_clients:
            if client_id in self.send_model_ids:
                self.request_start_train(client_id)
        self.get_logger().info("Waiting for all client training results...")

        start_time = time.time()
        send_model_nums = len(self.send_model_ids)
        while rclpy.ok() and send_model_nums!=0 and any(not fut.done() for fut, _ in self.train_futures.values()):
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - start_time > self.timeout_train_model:
                self.get_logger().warning("Timeout waiting for all training to complete; continuing")
                break
        self.receive_model_ids = []
        for client_id, (future, cli) in self.train_futures.items():
            try:
                assert future.done()
                assert future.result().success
                self.handle_train_model_result(future)
                self.get_logger().info(f"Received training result from client {client_id}")
            except Exception as e:
                client_name = self.registered_clients[client_id]
                self.get_logger().warning(f"Error processing result from client {client_name}: {e}")
                cli.remove_pending_request(future)
                future.cancel()
            finally:
                self.destroy_client(cli)

        self.get_logger().info("All client training completed; continuing")

    def train(self, selected_clients=[]):
        self.get_logger().info(f"{self.node_name} needs to implement train()")


    def receive_models(self):

        self.uploaded_ids = []
        self.uploaded_weights = []
        self.uploaded_models = []
        tot_samples = 0

        active_clients = self.receive_model_ids

        if not (len(active_clients) > 0):
            self.get_logger().warning(f"{self.node_name} received no model feedback")
            return


        for client_id in active_clients:
            client_result = self.train_model_results[client_id]
            tot_samples += client_result['train_samples']
            self.uploaded_ids.append(client_id)
            self.uploaded_weights.append(client_result['train_samples'])
            self.uploaded_models.append(client_result['model'])


        for i, w in enumerate(self.uploaded_weights):
            self.uploaded_weights[i] = w / tot_samples


    def aggregate_parameters(self):
        assert (len(self.uploaded_models) > 0), f"Error: {self.node_name} received zero models"

        self.global_model = copy.deepcopy(self.uploaded_models[0])
        for param in self.global_model.parameters():
            param.data.zero_()


        for w, client_model in zip(self.uploaded_weights, self.uploaded_models):
            self.add_parameters(w, client_model)


    def add_parameters(self, w, client_model):
        for server_param, client_param in zip(self.global_model.parameters(), client_model.parameters()):
            server_param.data += client_param.data.clone() * w

    def request_test_metrics(self, client_id):
        client_name = self.registered_clients[client_id]
        client_api = '/'+client_name+'/test_metrics'
        start_time = time.time()
        request_test_metrics_client = self.create_client(TestMetrics, client_api, callback_group=self.client_cb_group)
        while not request_test_metrics_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Waiting for client {client_id} test-evaluation service...")
            if time.time() - start_time > self.timeout_check:
                self.get_logger().warning(f"Timeout waiting for client {client_name} service response; continuing")
                return
        req = TestMetrics.Request()
        req.id = client_id
        req.node_name = client_name
        future = request_test_metrics_client.call_async(req)
        self.test_metrics_futures[client_id] = (future, request_test_metrics_client)


    def test_local_metrics(self):
        self.check_online()
        self.test_metrics_results = {}
        self.test_metrics_futures = {}

        num_samples = []
        tot_correct = []
        tot_auc = []
        ids = []

        for clients_id in self.online_clients_id:
            self.request_test_metrics(clients_id)

        start_time = time.time()
        while rclpy.ok() and len(self.online_clients_id)!=0  and any(not fut.done() for fut, _ in self.test_metrics_futures.values()):
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - start_time > self.timeout_metrics:
                self.get_logger().warning("Timeout waiting for all client test-evaluation responses; continuing")
                break

        for client_id, (future, cli) in self.test_metrics_futures.items():
            try:
                assert future.done()
                assert future.result().success
                ct = future.result().test_acc
                ns = future.result().test_num
                auc = future.result().auc
                ids.append(client_id)
                tot_correct.append(ct * 1.0)
                tot_auc.append(auc * ns)
                num_samples.append(ns)

            except Exception as e:
                client_name = self.registered_clients[client_id]
                self.get_logger().warning(f"Test evaluation failed on client {client_name}")
                cli.remove_pending_request(future)
                future.cancel()
            finally:
                self.destroy_client(cli)
        self.get_logger().info("All client test evaluations completed; continuing")
        return ids, num_samples, tot_correct, tot_auc

    def request_train_metrics(self, client_id):
        client_name = self.registered_clients[client_id]
        client_api = '/'+client_name+'/train_metrics'
        start_time = time.time()
        request_train_metrics_to_client = self.create_client(TrainMetrics, client_api, callback_group=self.client_cb_group)
        while not request_train_metrics_to_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f"Waiting for client {client_id} train-evaluation service...")
            if time.time() - start_time > self.timeout_check:
                client_name = self.registered_clients[client_id]
                self.get_logger().warning(f"Timeout waiting for client {client_name} service response; continuing")
                return
        req = TrainMetrics.Request()
        req.id = client_id
        req.node_name = client_name
        future = request_train_metrics_to_client.call_async(req)
        self.train_metrics_futures[client_id] = (future, request_train_metrics_to_client)

    def handle_train_metrics_result(self, future):
        train_metrics_res = {}
        response = future.result()
        client_id = response.node_id
        success = response.success
        try:
            train_metrics_res['losses'] = future.result().losses
            train_metrics_res['train_num'] = future.result().train_num
            self.train_metrics_results[client_id] = train_metrics_res
        except Exception as e:
            self.get_logger().warning(str(e))


    def train_local_metrics(self):
        self.check_online()
        self.train_metrics_results = {}
        self.train_metrics_futures = {}

        num_samples = []
        losses = []
        ids = []

        for clients_id in self.online_clients_id:
            self.request_train_metrics(clients_id)

        start_time = time.time()
        while rclpy.ok() and len(self.online_clients_id)!=0 and any(not fut.done() for fut, _ in self.train_metrics_futures.values()):
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - start_time > self.timeout_metrics:
                self.get_logger().warning("Timeout waiting for all client train-evaluation responses; continuing")
                break

        for client_id, (future, cli) in self.train_metrics_futures.items():
            try:
                assert future.done()
                assert future.result().success
                cl = future.result().losses
                ns = future.result().train_num
                ids.append(client_id)
                num_samples.append(ns)
                losses.append(cl * 1.0)

            except Exception as e:

                client_name = self.registered_clients[client_id]
                self.get_logger().warning(f"Train evaluation failed on client {client_name}")
                cli.remove_pending_request(future)
                future.cancel()
            finally:
                self.destroy_client(cli)
        self.get_logger().info("All client test evaluations completed; continuing")
        return ids, num_samples, losses

    def load_global_test_data(self, batch_size=None):
        if batch_size is None:
            batch_size = self.batch_size
        test_data = read_client_data(self.dataset, self.share_dir, "global", is_train=False, few_shot=self.few_shot)
        return DataLoader(test_data, batch_size, drop_last=False, shuffle=True)


    def test_global_metrics(self):
        self.get_logger().info("Starting aggregated model test evaluation (base)")
        testloaderfull = self.load_global_test_data()
        self.global_model.eval()
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
                output = self.global_model(x)

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

        test_acc_global = test_acc * 1.0 / test_num
        test_auc_global = auc * 1.0

        return test_acc_global, test_auc_global


    def evaluate(self, acc=None, loss=None):
        stats = self.test_local_metrics()
        stats_train = self.train_local_metrics()

        test_micro_acc_local = sum(stats[2]) * 1.0 / sum(stats[1])
        test_micro_auc_local = sum(stats[3]) * 1.0 / sum(stats[1])

        local_accs = [a / n for a, n in zip(stats[2], stats[1])]

        test_macro_acc_local = sum(local_accs) * 1.0 / len(local_accs)
        test_macro_acc_std_local = np.std(local_accs)
        test_macro_worst_local = np.min(local_accs)

        local_aucs = [a / n for a, n in zip(stats[3], stats[1])]
        test_macro_auc_local = sum(local_aucs) * 1.0 / len(local_aucs)
        test_macro_auc_std_local = np.std(local_aucs)
        test_macro_auc_worst_local = np.min(local_aucs)

        train_loss_local = sum(stats_train[2]) * 1.0 / sum(stats_train[1])
        test_acc_global, test_auc_global = self.test_global_metrics()

        self.rs_test_micro_acc_local.append(test_micro_acc_local)
        self.rs_test_micro_auc_local.append(test_micro_auc_local)
        self.rs_test_macro_acc_local.append(test_macro_acc_local)
        self.rs_test_macro_acc_std_local.append(test_macro_acc_std_local)
        self.rs_test_macro_acc_worst_local.append(test_macro_worst_local)
        self.rs_test_macro_auc_local.append(test_macro_auc_local)
        self.rs_test_macro_auc_std_local.append(test_macro_auc_std_local)
        self.rs_test_macro_auc_worst_local.append(test_macro_auc_worst_local)
        self.rs_train_loss_local.append(train_loss_local)
        self.rs_test_acc_global.append(test_acc_global)
        self.rs_test_auc_global.append(test_auc_global)

        print("Local Train Loss: {:.4f}".format(train_loss_local))
        print("Local Micro Accuracy: {:.4f}".format(test_micro_acc_local))
        print("Local Macro Accuracy: {:.4f}".format(test_macro_acc_local))
        print("Local Micro AUC: {:.4f}".format(test_micro_auc_local))
        print("Local Macro AUC: {:.4f}".format(test_macro_auc_local))
        print("Global  Test Accuracy: {:.4f}".format(test_acc_global))
        print("Global  Test AUC: {:.4f}".format(test_auc_global))


    def evaluate_final(self):
        self.get_logger().info(f"{self.node_name} needs to implement evaluate_final()")


    def print_(self, test_acc, test_auc, train_loss):
        print("Average Test Accuracy: {:.4f}".format(test_acc))
        print("Average Test AUC: {:.4f}".format(test_auc))
        print("Average Train Loss: {:.4f}".format(train_loss))


    def load_model(self):
        model_path = os.path.join("models", self.dataset)
        model_path = os.path.join(model_path, self.algorithm + "_server" + ".pt")
        assert (os.path.exists(model_path))
        self.global_model = torch.load(model_path)


    def model_exists(self):
        model_path = os.path.join("models", self.dataset)
        model_path = os.path.join(model_path, self.algorithm + "_server" + ".pt")
        return os.path.exists(model_path)

    def _safe_last(self, seq):
        return seq[-1] if seq else None