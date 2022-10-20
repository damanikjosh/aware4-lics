#!/bin/bash

docker exec -it cisim /bin/bash -c "source /root/px4_ros_com_ros2/install/setup.bash && python3 /root/scenario/main.py"