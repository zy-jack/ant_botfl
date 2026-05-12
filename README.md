# CRoWN-FL: Cross-Layer Federated Learning Simulation for Mobile Robotic Collaboration

CRoWN-FL is a cross-layer simulation framework for evaluating federated learning in mobile robotic systems over wireless networks. It couples robot mobility, time-varying wireless channel modeling, packet-level model transmission, and FL execution into a round-level closed loop, so that each aggregation round reflects the actual availability of model updates under realistic communication conditions. The framework is designed for studying how mobility, channel dynamics, model workload, and wireless delivery constraints jointly affect FL convergence and final model performance.

## Demo

The following video demonstrates the round-level co-simulation workflow of CRoWN-FL, including robot mobility, wireless transmission, and federated learning execution.

<p align="center">
  <img src="assets/demo.gif" width="850" alt="CRoWN-FL demo" />
</p>

## Key Features

- **Cross-layer co-simulation**: integrates ROS 2-based robot/FL control, MATLAB-based wireless channel trace generation, and ns-3-based packet-level LTE simulation.
- **Round-level FL workflow**: local training, model serialization, wireless transmission, delivery report parsing, and completion-aware aggregation are executed in a closed loop.
- **Mobility-aware wireless modeling**: robot trajectories are mapped to time-varying channel states, enabling mobility-driven path loss and fading effects.
- **Packet-level model delivery**: FL model updates are packetized and transmitted through the wireless network stack instead of being assumed to arrive ideally.
- **Extensible FL algorithms and models**: the current code provides a FedAvg reference implementation, and new algorithms/models can be added through the existing client/server and configuration interfaces.

## Prerequisites

The following software is required:

- **ROS 2 Humble**
- **ns-3 3.45**
- **MATLAB R2021**
- **XTerm**
- **Python dependencies** listed in `robot/requirements.txt`

A Linux environment with ROS 2 Humble support is recommended.

## Installation

Clone the repository:

```bash
git clone https://github.com/zy-jack/ant_botfl.git
cd ant_botfl
```

Install Python dependencies:

```bash
cd robot
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Build the ROS 2 FL workspace:

```bash
bash run/setup.sh
```

Prepare your local ns-3 and ROS configuration paths before running the simulation.

## Quick Start

Run the project with:

```bash
./run.sh \
  --ns3-root /path/to/ns3_config \
  --ros-root /path/to/ros_config
```

Arguments:

- `--ns3-root`: path to the ns-3-side configuration or workspace used by the network simulation.
- `--ros-root`: path to the ROS-side configuration or workspace used by the FL/robot simulation.

Adjust the paths according to your local environment.

## Advanced Usage

### 1. Configure Federated Learning Parameters

FL parameters are configured under:

```text
robot/rosfl_code/rosfl/datas/config/
```

Create a new directory for each algorithm and add the corresponding configuration files. The project provides a FedAvg example under:

```text
robot/rosfl_code/rosfl/datas/config/FedAvg/
├── client.yaml
└── server.yaml
```

You can use this directory as a reference when adding new algorithms or experiment settings.

### 2. Generate FL Datasets

Dataset generation scripts are located in:

```text
robot/rosfl_code/rosfl/rosfl/dataset/
```

Examples provided by the project include:

```bash
cd robot/rosfl_code/rosfl/rosfl/dataset
python generate_Cifar10.py
python generate_Cifar100.py
python generate_SVHN.py
```

For additional datasets, implement a new `generate_*.py` script by following the structure of the existing dataset-generation files.

### 3. Replace the Experimental Model

Model selection and registration are handled in:

```text
robot/rosfl_code/rosfl/rosfl/utils/rosconfig_utils.py
```

To use another model:

1. Check whether the required model is already supported in `rosconfig_utils.py`.
2. If it is supported, specify the model in the FL configuration files.
3. If it is not supported, add the model registration or loading logic in `rosconfig_utils.py`, then reference it from the configuration.

### 4. Add a New FL Algorithm

Use the FedAvg implementation as the reference. Client and server implementations are located under:

```text
robot/rosfl_code/rosfl/rosfl/ros_fl/
├── clients/
└── servers/
```

To add a new algorithm:

1. Create new client and server classes based on the FedAvg example.
2. Reconstruct or override the `train` function according to the algorithm logic.
3. Add any required aggregation, scheduling, or update-processing functions.
4. Create matching configuration files under `robot/rosfl_code/rosfl/datas/config/<AlgorithmName>/`.
5. Update configuration loading logic if the new algorithm requires additional fields.

### 5. Configure Wireless-Side Parameters

Wireless and network simulation parameters should be configured in the `robot_fl_simulation_config` files under:

```text
network/
├── robot_fl_simulation_config.cc
└── robot_fl_simulation_config.h
```

Use these files to configure LTE/network parameters, robot/network simulation settings, packet-level delivery behavior, and other wireless-side experiment controls.

## Typical Workflow

```text
1. Configure FL parameters
   └── robot/rosfl_code/rosfl/datas/config/<AlgorithmName>/

2. Generate or prepare datasets
   └── robot/rosfl_code/rosfl/rosfl/dataset/generate_*.py

3. Configure models and algorithms
   ├── robot/rosfl_code/rosfl/rosfl/utils/rosconfig_utils.py
   └── robot/rosfl_code/rosfl/rosfl/ros_fl/

4. Configure wireless/network settings
   └── network/robot_fl_simulation_config.*

5. Run the simulation
   └── ./run.sh --ns3-root ... --ros-root ...
```

## Roadmap

Future updates will provide extended network configurations, including support for 5G scenarios and additional wireless-side configurations.

## Citation

If you use this project in academic work, please cite the CRoWN-FL paper after publication. The BibTeX entry will be updated once the paper information is available.

```bibtex
@misc{yan2026crownfl,
  title  = {CRoWN-FL: A Cross-Layer Federated Learning Simulation Framework for Mobile Robotic Collaboration over Wireless Networks},
  author = {Zhenyu Yan and Zhenhui Yuan and Tianhao Peng and Ziyang Chen and Guangjing He and Yuming Zhang and Peng Li and Fei Song},
  year   = {2026},
  note   = {Submitted to IEEE GLOBECOM, under review}
}
```

## License

This project is released under the AGPL-3.0 License. See the LICENSE file for details.

## Contact

For questions or issues, please open an issue in this repository.

