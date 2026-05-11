import numpy as np
import os
import sys
import random
import torch
import torchvision
import torchvision.transforms as transforms
from utils.dataset_utils import check, separate_data, split_data, save_file, global_split, classwise_subsample

random.seed(1)
np.random.seed(1)
num_clients = 10
dir_path = "../../datas/dataset/Cifar100/"

NIID = True
BALANCE = False
PARTITION = 'dir'

"""
pat
dir
exdir
"""


def generate_dataset(dir_path, num_clients, niid, balance, partition):
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)


    config_path = dir_path + "config.json"
    train_path = dir_path + "train/"
    test_path = dir_path + "test/"

    if check(config_path, train_path, test_path, num_clients, niid, balance, partition):
        return


    transform = transforms.Compose(
        [transforms.ToTensor(), transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))])

    trainset = torchvision.datasets.CIFAR100(
        root=dir_path+"rawdata", train=True, download=True, transform=transform)
    testset = torchvision.datasets.CIFAR100(
        root=dir_path+"rawdata", train=False, download=True, transform=transform)
    trainloader = torch.utils.data.DataLoader(
        trainset, batch_size=len(trainset.data), shuffle=False)
    testloader = torch.utils.data.DataLoader(
        testset, batch_size=len(testset.data), shuffle=False)

    for _, train_data in enumerate(trainloader, 0):
        trainset.data, trainset.targets = train_data
    for _, test_data in enumerate(testloader, 0):
        testset.data, testset.targets = test_data

    dataset_image = []
    dataset_label = []

    dataset_image.extend(trainset.data.cpu().detach().numpy())
    dataset_image.extend(testset.data.cpu().detach().numpy())
    dataset_label.extend(trainset.targets.cpu().detach().numpy())
    dataset_label.extend(testset.targets.cpu().detach().numpy())
    dataset_image = np.array(dataset_image)
    dataset_label = np.array(dataset_label)

    print(f"[Original Full Dataset] Size: {len(dataset_label)}")
    print(f"[Original Full Dataset] Unique labels: {np.unique(dataset_label)}")


    dataset_image, dataset_label = classwise_subsample(
        (dataset_image, dataset_label)
    )

    num_classes = len(np.unique(dataset_label))
    print(f'Number of classes after subsample: {num_classes}')


    global_train_data, global_test_data = global_split((dataset_image, dataset_label))


    X, y, statistic = separate_data(
        global_train_data,
        num_clients,
        num_classes,
        niid,
        balance,
        partition,
        class_per_client=2
    )


    train_data, local_test_data = split_data(X, y)


    save_file(
        config_path,
        train_path,
        test_path,
        train_data,
        local_test_data,
        num_clients,
        num_classes,
        statistic,
        global_test_data=global_test_data,
        niid=niid,
        balance=balance,
        partition=partition
    )


if __name__ == "__main__":

    niid = NIID
    balance = BALANCE
    partition = PARTITION
    generate_dataset(dir_path, num_clients, niid, balance, partition)