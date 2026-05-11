import numpy as np
import os
import random
import torch
import torchvision
import torchvision.transforms as transforms

from utils.dataset_utils import (
    check,
    separate_data,
    split_data,
    save_file,
    global_split,
    classwise_subsample,
)

random.seed(1)
np.random.seed(1)
torch.manual_seed(1)

num_clients = 10
dir_path = "../../datas/dataset/SVHN/"

NIID = True
BALANCE = False
PARTITION = 'dir'

"""
pat
dir
exdir
"""


USE_EXTRA = False


LOAD_BATCH_SIZE = 512


def load_split_to_numpy(root_path, split, transform, batch_size=512):
    """
    Read one SVHN split in batches and concatenate it into numpy arrays:
    - images: [N, 3, 32, 32]
    - labels: [N]
    """
    dataset = torchvision.datasets.SVHN(
        root=root_path,
        split=split,
        download=True,
        transform=transform
    )

    loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=False,
        num_workers=0
    )

    image_list = []
    label_list = []

    for images, labels in loader:
        image_list.append(images.cpu().numpy())
        label_list.append(labels.cpu().numpy())

    images_np = np.concatenate(image_list, axis=0)
    labels_np = np.concatenate(label_list, axis=0)

    return images_np, labels_np


def generate_dataset(dir_path, num_clients, niid, balance, partition):
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)


    config_path = dir_path + "config.json"
    train_path = dir_path + "train/"
    test_path = dir_path + "test/"

    if check(config_path, train_path, test_path, num_clients, niid, balance, partition):
        return


    transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
    ])

    dataset_image = []
    dataset_label = []


    train_images, train_labels = load_split_to_numpy(
        root_path=dir_path + "rawdata",
        split="train",
        transform=transform,
        batch_size=LOAD_BATCH_SIZE
    )
    dataset_image.append(train_images)
    dataset_label.append(train_labels)


    test_images, test_labels = load_split_to_numpy(
        root_path=dir_path + "rawdata",
        split="test",
        transform=transform,
        batch_size=LOAD_BATCH_SIZE
    )
    dataset_image.append(test_images)
    dataset_label.append(test_labels)


    if USE_EXTRA:
        extra_images, extra_labels = load_split_to_numpy(
            root_path=dir_path + "rawdata",
            split="extra",
            transform=transform,
            batch_size=LOAD_BATCH_SIZE
        )
        dataset_image.append(extra_images)
        dataset_label.append(extra_labels)

    dataset_image = np.concatenate(dataset_image, axis=0)
    dataset_label = np.concatenate(dataset_label, axis=0)

    print(f"[Original Full Dataset] Size: {len(dataset_label)}")
    print(f"[Original Full Dataset] Unique labels: {np.unique(dataset_label)}")


    dataset_image, dataset_label = classwise_subsample(
        (dataset_image, dataset_label)
    )

    num_classes = len(np.unique(dataset_label))
    print(f"Number of classes after subsample: {num_classes}")


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