deactivate
source ./ros_fl_env/bin/activate
clear
cd rosfl_code
clear
source install/setup.bash

ros2 run rosfl exec_client_node -- --node_name client_9