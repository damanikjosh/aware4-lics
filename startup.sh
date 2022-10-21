#!/bin/bash

source /root/px4_ros_com_ros2/install/setup.bash
cp -r /root/scenario/models/* /root/PX4-Autopilot/Tools/sitl_gazebo/models/
cp /root/scenario/main.world /root/PX4-Autopilot/Tools/sitl_gazebo/worlds/
python3 /root/scenario/init.py
sleep 3
cd /root/PX4-Autopilot/Tools &&
./qtr_multiple_run.sh -f /root/scenario/initial.config -w main &
MicroXRCEAgent udp4 -p 5000
