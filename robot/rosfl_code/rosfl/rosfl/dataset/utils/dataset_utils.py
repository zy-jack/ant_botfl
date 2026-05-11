import os
import ujson
import numpy as np
from sklearn.model_selection import train_test_split
from torch.utils.data import Dataset
from PIL import Image


batch_size = 10


global_train_ratio = 0.90


local_train_ratio = 0.8


class_subset_ratio = 1.0

alpha = 0.1


def check(config_path, train_path, test_path, num_clients, niid=False,
          balance=True, partition=None):

    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = ujson.load(f)

        if config.get('num_clients') == num_clients and\
           config.get('non_iid') == niid and\
           config.get('balance') == balance and\
           config.get('partition') == partition and\
           config.get('alpha') == alpha and\
           config.get('batch_size') == batch_size and\
           config.get('global_train_ratio') == global_train_ratio and\
           config.get('local_train_ratio') == local_train_ratio and\
           config.get('class_subset_ratio') == class_subset_ratio:
            print("\nDataset already generated.\n")
            return True

    dir_path = os.path.dirname(train_path)
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

    dir_path = os.path.dirname(test_path)
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

    return False


def classwise_subsample(data, subset_ratio=class_subset_ratio, min_keep_per_class=2, random_state=1):
    """
    Perform proportional per-class sampling on the full dataset:
    - Keep a subset_ratio fraction of samples from each class
    - Run subsequent global, federated, and local splits only on this subset
    """
    if not (0 < subset_ratio <= 1):
        raise ValueError("subset_ratio must be in (0, 1].")

    dataset_content, dataset_label = data
    rng = np.random.RandomState(random_state)

    unique_labels = np.unique(dataset_label)
    selected_indices = []

    print(f"[Class-wise Subsample] Keep ratio per class = {subset_ratio}")

    statistic_before = []
    statistic_after = []

    for lbl in unique_labels:
        idx = np.where(dataset_label == lbl)[0]
        rng.shuffle(idx)

        keep_n = int(round(len(idx) * subset_ratio))
        keep_n = max(min_keep_per_class, keep_n)
        keep_n = min(keep_n, len(idx))

        chosen = idx[:keep_n]
        selected_indices.append(chosen)

        statistic_before.append((int(lbl), int(len(idx))))
        statistic_after.append((int(lbl), int(keep_n)))

        print(f"Class {int(lbl):>2}: {len(idx)} -> {keep_n}")

    selected_indices = np.concatenate(selected_indices, axis=0)
    rng.shuffle(selected_indices)

    sampled_content = dataset_content[selected_indices]
    sampled_label = dataset_label[selected_indices]

    print("-" * 50)
    print(f"[Class-wise Subsample] Total size: {len(dataset_label)} -> {len(sampled_label)}")
    print("[Class-wise Subsample] Before:", statistic_before)
    print("[Class-wise Subsample] After :", statistic_after)
    print("-" * 50)

    return sampled_content, sampled_label


def global_split(data):
    """
    Split the per-class sampled subset with global_train_ratio into:
    - global_train_data: (X_train, y_train)
    - global_test_data:  (X_test,  y_test)
    """
    dataset_content, dataset_label = data

    try:
        X_train, X_test, y_train, y_test = train_test_split(
            dataset_content,
            dataset_label,
            train_size=global_train_ratio,
            shuffle=True,
            stratify=dataset_label,
            random_state=1
        )
    except ValueError as e:
        print(f"[Global Split] stratify failed: {e}")
        print("[Global Split] Fallback to non-stratified split.")
        X_train, X_test, y_train, y_test = train_test_split(
            dataset_content,
            dataset_label,
            train_size=global_train_ratio,
            shuffle=True,
            random_state=1
        )

    global_train_data = (X_train, y_train)
    global_test_data = (X_test, y_test)

    print(f"[Global Split] Train size: {len(y_train)}, Test size: {len(y_test)}")

    unique_labels = np.unique(y_test)
    statistic_global = []
    for lbl in unique_labels:
        statistic_global.append((int(lbl), int(sum(y_test == lbl))))

    print(f"Global test\t Size of data: {len(y_test)}\t Labels: ", unique_labels)
    print("\t\t Samples of labels: ", [i for i in statistic_global])
    print("-" * 50)

    return global_train_data, global_test_data


def separate_data(data, num_clients, num_classes, niid=False,
                  balance=False, partition=None, class_per_client=None):
    """
    Note: data is global_train_data, the global training set,
    not the full dataset.
    """
    X = [[] for _ in range(num_clients)]
    y = [[] for _ in range(num_clients)]
    statistic = [[] for _ in range(num_clients)]

    dataset_content, dataset_label = data


    least_samples = int(min(
        batch_size / (1 - local_train_ratio),
        len(dataset_label) / num_clients / 2
    ))

    dataidx_map = {}

    if not niid:
        partition = 'pat'
        class_per_client = num_classes

    if partition == 'pat':
        idxs = np.array(range(len(dataset_label)))
        idx_for_each_class = []
        for i in range(num_classes):
            idx_for_each_class.append(idxs[dataset_label == i])

        class_num_per_client = [class_per_client for _ in range(num_clients)]
        for i in range(num_classes):
            selected_clients = []
            for client in range(num_clients):
                if class_num_per_client[client] > 0:
                    selected_clients.append(client)
            if len(selected_clients) == 0:
                break
            selected_clients = selected_clients[:int(
                np.ceil((num_clients / num_classes) * class_per_client)
            )]

            num_all_samples = len(idx_for_each_class[i])
            num_selected_clients = len(selected_clients)
            num_per = num_all_samples / num_selected_clients
            if balance:
                num_samples = [int(num_per) for _ in range(num_selected_clients - 1)]
            else:
                num_samples = np.random.randint(
                    max(num_per / 10, least_samples / num_classes),
                    num_per,
                    num_selected_clients - 1
                ).tolist()
            num_samples.append(num_all_samples - sum(num_samples))

            idx = 0
            for client, num_sample in zip(selected_clients, num_samples):
                if client not in dataidx_map.keys():
                    dataidx_map[client] = idx_for_each_class[i][idx:idx + num_sample]
                else:
                    dataidx_map[client] = np.append(
                        dataidx_map[client],
                        idx_for_each_class[i][idx:idx + num_sample],
                        axis=0
                    )
                idx += num_sample
                class_num_per_client[client] -= 1

    elif partition == "dir":
        min_size = 0
        K = num_classes
        N = len(dataset_label)

        try_cnt = 1
        while min_size < least_samples:
            if try_cnt > 1:
                print(f'Client data size does not meet the minimum requirement {least_samples}. '
                      f'Try allocating again for the {try_cnt}-th time.')

            idx_batch = [[] for _ in range(num_clients)]
            for k in range(K):
                idx_k = np.where(dataset_label == k)[0]
                np.random.shuffle(idx_k)
                proportions = np.random.dirichlet(np.repeat(alpha, num_clients))
                proportions = np.array([
                    p * (len(idx_j) < N / num_clients)
                    for p, idx_j in zip(proportions, idx_batch)
                ])
                proportions = proportions / proportions.sum()
                proportions = (np.cumsum(proportions) * len(idx_k)).astype(int)[:-1]
                idx_batch = [
                    idx_j + idx.tolist()
                    for idx_j, idx in zip(idx_batch, np.split(idx_k, proportions))
                ]
                min_size = min([len(idx_j) for idx_j in idx_batch])
            try_cnt += 1

        for j in range(num_clients):
            dataidx_map[j] = idx_batch[j]

    elif partition == 'exdir':
        r'''This strategy comes from https://arxiv.org/abs/2311.03154
        See details in https://github.com/TsingZ0/PFLlib/issues/139

        This version in PFLlib is slightly different from the original version
        Some changes are as follows:
        n_nets -> num_clients, n_class -> num_classes
        '''
        C = class_per_client

        '''The first level: allocate labels to clients'''
        min_size_per_label = 0
        min_require_size_per_label = max(C * num_clients // num_classes // 2, 1)
        if min_require_size_per_label < 1:
            raise ValueError
        clientidx_map = {}
        while min_size_per_label < min_require_size_per_label:

            for k in range(num_classes):
                clientidx_map[k] = []

            for i in range(num_clients):
                labelidx = np.random.choice(range(num_classes), C, replace=False)
                for k in labelidx:
                    clientidx_map[k].append(i)
            min_size_per_label = min([len(clientidx_map[k]) for k in range(num_classes)])

        '''The second level: allocate data idx'''
        dataidx_map = {}
        y_train = dataset_label
        min_size = 0
        min_require_size = 10
        K = num_classes
        N = len(y_train)
        print("\n*****clientidx_map*****")
        print(clientidx_map)
        print("\n*****Number of clients per label*****")
        print([len(clientidx_map[i]) for i in range(len(clientidx_map))])


        while min_size < min_require_size:
            idx_batch = [[] for _ in range(num_clients)]

            for k in range(K):
                idx_k = np.where(y_train == k)[0]
                np.random.shuffle(idx_k)
                proportions = np.random.dirichlet(np.repeat(alpha, num_clients))
                proportions = np.array([
                    p * (len(idx_j) < N / num_clients and j in clientidx_map[k])
                    for j, (p, idx_j) in enumerate(zip(proportions, idx_batch))
                ])
                proportions = proportions / proportions.sum()
                proportions = (np.cumsum(proportions) * len(idx_k)).astype(int)[:-1]

                if proportions[-1] != len(idx_k):
                    for w in range(clientidx_map[k][-1], num_clients - 1):
                        proportions[w] = len(idx_k)

                idx_batch = [
                    idx_j + idx.tolist()
                    for idx_j, idx in zip(idx_batch, np.split(idx_k, proportions))
                ]
                min_size = min([len(idx_j) for idx_j in idx_batch])

        for j in range(num_clients):
            np.random.shuffle(idx_batch[j])
            dataidx_map[j] = idx_batch[j]

    else:
        raise NotImplementedError


    for client in range(num_clients):
        idxs = dataidx_map[client]
        X[client] = dataset_content[idxs]
        y[client] = dataset_label[idxs]

        for i in np.unique(y[client]):
            statistic[client].append((int(i), int(sum(y[client] == i))))

    del data

    for client in range(num_clients):
        print(f"Client {client}\t Size of data: {len(X[client])}\t Labels: ", np.unique(y[client]))
        print(f"\t\t Samples of labels: ", [i for i in statistic[client]])
        print("-" * 50)

    return X, y, statistic


def split_data(X, y):
    """
    Within each client, split that client's data with local_train_ratio into:
    - Local training set
    - Local test set
    """
    train_data, test_data = [], []
    num_samples = {'train': [], 'test': []}

    for i in range(len(y)):
        X_train, X_test, y_train, y_test = train_test_split(
            X[i], y[i],
            train_size=local_train_ratio,
            shuffle=True,
            random_state=1
        )

        train_data.append({'x': X_train, 'y': y_train})
        num_samples['train'].append(len(y_train))
        test_data.append({'x': X_test, 'y': y_test})
        num_samples['test'].append(len(y_test))

    print("Total number of samples:", sum(num_samples['train'] + num_samples['test']))
    print("The number of local train samples per client:", num_samples['train'])
    print("The number of local test samples per client:", num_samples['test'])
    print()

    del X, y
    return train_data, test_data


def save_file(config_path, train_path, test_path,
              train_data, test_data, num_clients,
              num_classes, statistic, global_test_data=None,
              niid=False, balance=True, partition=None):
    config = {
        'num_clients': num_clients,
        'num_classes': num_classes,
        'non_iid': niid,
        'balance': balance,
        'partition': partition,
        'Size of samples for labels in clients': statistic,
        'alpha': alpha,
        'batch_size': batch_size,
        'global_train_ratio': global_train_ratio,
        'local_train_ratio': local_train_ratio,
        'class_subset_ratio': class_subset_ratio,
    }

    print("Saving to disk.\n")


    for idx, train_dict in enumerate(train_data):
        with open(train_path + str(idx) + '.npz', 'wb') as f:
            np.savez_compressed(f, data=train_dict)


    for idx, test_dict in enumerate(test_data):
        with open(test_path + str(idx) + '.npz', 'wb') as f:
            np.savez_compressed(f, data=test_dict)


    if global_test_data is not None:
        X_gtest, y_gtest = global_test_data
        global_test_dict = {'x': X_gtest, 'y': y_gtest}
        with open(test_path + 'global.npz', 'wb') as f:
            np.savez_compressed(f, data=global_test_dict)

    with open(config_path, 'w') as f:
        ujson.dump(config, f)

    print("Finish generating dataset.\n")


class ImageDataset(Dataset):
    def __init__(self, dataframe, image_folder, transform=None):
        """
        Args:
            dataframe (pd.DataFrame): DataFrame containing file names
            image_folder (str): Path to the folder containing the images
            transform (callable, optional): Optional transform to be applied to the image
        """
        self.dataframe = dataframe
        self.image_folder = image_folder
        self.transform = transform

    def __len__(self):
        return len(self.dataframe)

    def __getitem__(self, idx):

        img_name = self.dataframe.iloc[idx]['file_name']
        img_label = self.dataframe.iloc[idx]['class']
        img_path = os.path.join(self.image_folder, img_name)


        image = Image.open(img_path).convert('RGB')

        if self.transform:
            image = self.transform(image)

        return image, img_label