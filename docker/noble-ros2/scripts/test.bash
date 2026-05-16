#!/bin/bash
set -e

# setup env
source env/bin/activate
source /opt/ros/jazzy/setup.bash
source /opt/xbot/setup.sh
source ~/test_ws/setup.bash 

# run tests
cd ~/test_ws/build/xbot2_mujoco
ctest --output-on-failure