deactivate
source ./venv/bin/activate
clear
cd rosfl_code
rm -rf build/
rm -rf log/
rm -rf install/
clear
python -m colcon build
cd ../