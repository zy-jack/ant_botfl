import copy
import yaml
import torch
import argparse
import os
import time
import warnings
import numpy as np
import torchvision
import logging

from types import SimpleNamespace

from rosfl.ros_fl.trainmodel.models import *
from rosfl.ros_fl.trainmodel.bilstm import *
from rosfl.ros_fl.trainmodel.resnet import *
from rosfl.ros_fl.trainmodel.alexnet import *
from rosfl.ros_fl.trainmodel.mobilenet_v2 import *
from rosfl.ros_fl.trainmodel.transformer import *

def load_rosconfig(path):
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    args = SimpleNamespace(**cfg)
    return args

def configure_model(args):
    model_str = args.model
    dataset_str = args.dataset
    model_target = None
    if model_str == "MLR":
        if "MNIST" in dataset_str:
            model_target = Mclr_Logistic(1 * 28 * 28, num_classes=args.num_classes).to(args.device)
        elif "Cifar10" in dataset_str:
            model_target = Mclr_Logistic(3 * 32 * 32, num_classes=args.num_classes).to(args.device)
        else:
            model_target = Mclr_Logistic(60, num_classes=args.num_classes).to(args.device)

    elif model_str == "CNN":
        if "MNIST" in dataset_str:
            model_target = FedAvgCNN(in_features=1, num_classes=args.num_classes, dim=1024).to(args.device)
        elif "Cifar10" in dataset_str:
            model_target = FedAvgCNN(in_features=3, num_classes=args.num_classes, dim=1600).to(args.device)
        elif "Omniglot" in dataset_str:
            model_target = FedAvgCNN(in_features=1, num_classes=args.num_classes, dim=33856).to(args.device)

        elif "Digit5" in dataset_str:
            model_target = Digit5CNN().to(args.device)
        else:
            model_target = FedAvgCNN(in_features=3, num_classes=args.num_classes, dim=10816).to(args.device)

    elif model_str == "DNN":
        if "MNIST" in dataset_str:
            model_target = DNN(1 * 28 * 28, 100, num_classes=args.num_classes).to(args.device)
        elif "Cifar10" in dataset_str:
            model_target = DNN(3 * 32 * 32, 100, num_classes=args.num_classes).to(args.device)
        else:
            model_target = DNN(60, 20, num_classes=args.num_classes).to(args.device)

    elif model_str == "ResNet18":
        if "EMNIST" in dataset_str:
            model_target = torchvision.models.resnet18(
                weights=None,
                num_classes=args.num_classes
            )

            model_target.conv1 = nn.Conv2d(
                1, 64, kernel_size=3, stride=1, padding=1, bias=False
            )

            model_target.maxpool = nn.Identity()
            model_target = model_target.to(args.device)
        else:
            model_target = torchvision.models.resnet18(weights=None, num_classes=args.num_classes).to(args.device)

    elif model_str == "ResNet8":
        model_target = resnet8(num_classes=args.num_classes).to(args.device)

    elif model_str == "ResNet10":
        model_target = resnet10(num_classes=args.num_classes).to(args.device)

    elif model_str == "ResNet34":
        model_target = torchvision.models.resnet34(weights=None, num_classes=args.num_classes).to(args.device)

    elif model_str == "ResNet50":
        model_target = resnet50(num_classes=args.num_classes).to(args.device)

    elif model_str == "AlexNet":
        model_target = alexnet(pretrained=False, num_classes=args.num_classes).to(args.device)

    elif model_str == "GoogleNet":
        model_target = torchvision.models.googlenet(pretrained=False, aux_logits=False,
                                                  num_classes=args.num_classes).to(args.device)
    elif model_str == "MobileNet":
        model_target = mobilenet_v2(pretrained=False, num_classes=args.num_classes).to(args.device)

    elif model_str == "LSTM":
        model_target = LSTMNet(hidden_dim=args.feature_dim, vocab_size=args.vocab_size, num_classes=args.num_classes).to(
            args.device)

    elif model_str == "BiLSTM":
        model_target = BiLSTM_TextClassification(input_size=args.vocab_size, hidden_size=args.feature_dim,
                                               output_size=args.num_classes, num_layers=1,
                                               embedding_dropout=0, lstm_dropout=0, attention_dropout=0,
                                               embedding_length=args.feature_dim).to(args.device)
    elif model_str == "fastText":
        model_target = fastText(hidden_dim=args.feature_dim, vocab_size=args.vocab_size, num_classes=args.num_classes).to(
            args.device)

    elif model_str == "TextCNN":
        model_target = TextCNN(hidden_dim=args.feature_dim, max_len=args.max_len, vocab_size=args.vocab_size,
                             num_classes=args.num_classes).to(args.device)

    elif model_str == "Transformer":
        model_target = TransformerModel(ntoken=args.vocab_size, d_model=args.feature_dim, nhead=8, nlayers=2,
                                      num_classes=args.num_classes, max_len=args.max_len).to(args.device)
    elif model_str == "AmazonMLP":
        model_target = AmazonMLP().to(args.device)

    elif model_str == "HARCNN":
        if dataset_str == 'HAR':
            model_target = HARCNN(9, dim_hidden=1664, num_classes=args.num_classes, conv_kernel_size=(1, 9),
                                pool_kernel_size=(1, 2)).to(args.device)
        elif dataset_str == 'PAMAP2':
            model_target = HARCNN(9, dim_hidden=3712, num_classes=args.num_classes, conv_kernel_size=(1, 9),
                                pool_kernel_size=(1, 2)).to(args.device)
    else:
        raise NotImplementedError

    return model_target

def configure_algorithm(args):
    args_target = copy.deepcopy(args)
    if args.algorithm == "FedAvg":
        args_target.head = copy.deepcopy(args.model.fc)
        args_target.model.fc = nn.Identity()
        args_target.model = BaseHeadSplit(args_target.model, args_target.head)
    elif args.algorithm == "FedProx":
        pass
    else:
        raise NotImplementedError
    return args_target
