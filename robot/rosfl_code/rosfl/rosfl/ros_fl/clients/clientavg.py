import argparse

import numpy as np
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rosfl.ros_fl.clients.clientbase import Client

class clientAVG(Client):
    def __init__(self, package_name, node_name='ros_client', node_config='FedAvg', **kwargs):
        super().__init__(package_name, node_name, node_config, **kwargs)

    def train(self):
        train_loader = self.load_train_data()
        start_time = time.time()
        self.model.train()
        max_local_epochs = self.local_epochs
        if self.train_slow:
            max_local_epochs = np.random.randint(1, max_local_epochs // 2)
        for epoch in range(max_local_epochs):
            for i, (x, y) in enumerate(train_loader):
                if type(x) == type([]):
                    x[0] = x[0].to(self.device)
                else:
                    x = x.to(self.device)
                y = y.to(self.device)
                if self.train_slow:
                    time.sleep(0.1 * np.abs(np.random.rand()))
                output = self.model(x)
                loss = self.loss(output, y)
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
        if self.learning_rate_decay:
            self.learning_rate_scheduler.step()
        self.train_time_cost['num_rounds'] += 1
        self.train_time_cost['total_cost'] += time.time() - start_time

        self.test_metrics_trained()
        self.train_metrics_trained()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--node_name', type=str, help='Robot name')
    args = parser.parse_args()
    print(args)
    node_name = args.node_name


    rclpy.init()
    node = clientAVG(package_name='rosfl', node_name=node_name)
    node.request_register_to_server()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Program stopped by user (KeyboardInterrupt)")
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print("Cleanup completed. Exiting.")