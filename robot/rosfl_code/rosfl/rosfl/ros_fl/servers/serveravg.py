import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rosfl.ros_fl.servers.serverbase import Server

class FedAvg(Server):
    def __init__(self, package_name, node_name='ros_server', node_config='FedAvg', **kwargs):
        super().__init__(package_name, node_name, node_config, **kwargs)
        times = 0
        self.set_slow_clients(self.num_clients)
        self.generate_clients_parameters(self.num_clients)
        self.Budget = []
        self.round_index = 0

    def train(self, selected_clients=[]):
        start_t = time.time()
        finish_time = 0
        self.selected_clients = self.select_clients(selected_clients)
        try:
            self.selected_clients = self.select_clients()
            self.send_models(self.round_index)
            send_t = time.time()
            print('send model time cost:', send_t - start_t)
            if self.round_index % self.eval_gap == 0:
                print(f"\n-------------Round number: {self.round_index}-------------")
                print("\nEvaluate global model")
                self.evaluate()
            evaluate_time = time.time()
            print('evaluate model time cost:', evaluate_time - send_t)
            self.start_train(self.round_index)
            train_t = time.time()
            print('train model time cost:', train_t - evaluate_time)
            self.receive_models()
            self.aggregate_parameters()
            finish_time = time.time() - start_t
        except Exception as e:
            finish_time = time.time() - start_t + self.time_error
            self.get_logger().warning(str(e))
        finally:
            self.Budget.append(finish_time)
            print('-'*25, 'time cost', '-'*25, finish_time)
            self.save_result()
            self.selected_clients = []
            self.round_index += 1
            if self.round_index >= self.global_rounds:
                self.evaluate_final()

    def evaluate_final(self):
        print("\nBest accuracy.")
        print(max(self.rs_test_acc))
        print("\nAverage time cost per round.")
        print(sum(self.Budget[1:]) / len(self.Budget[1:]))


def main():
    rclpy.init()
    node = FedAvg(package_name='rosfl')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Program stopped by user (KeyboardInterrupt)")
    finally:
        print("Cleaning up resources...")
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        print("Cleanup completed. Exiting.")