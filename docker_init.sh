#!/bin/bash

docker run --rm\
    --gpus all\
    -it\
    --privileged\
    --env=DISPLAY=$DISPLAY\
    --env=LOCAL_USER_ID="$(id -u)"\
    --env="QT_X11_NO_MITSHM=1"\
    --device /dev:/dev:rw\
    --hostname $(hostname)\
    -p 14556:14556/udp\
    -u root\
    -v /tmp/.X11-unix:/tmp/.X11-unix\
    -v /home/lig/dockerVolume/:/mnt\
    -v $(pwd)/sitl_target.cmake:/root/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake\
    -v $(pwd)/test_process.sh:/root/test_process.sh\
    --shm-size=16G\
    --network host\
    --name cisim\
    -v $(pwd)/Illegal_Fishing/models/smooth:/root/PX4-Autopilot/Tools/sitl_gazebo/models/smooth\
    -v $(pwd)/Illegal_Fishing/worlds/smooth.world:/root/PX4-Autopilot/Tools/sitl_gazebo/worlds/smooth.world\
    -v $(pwd)/scenarios/callback_scenario.py:/root/px4_ros_com_ros2/build/rtps_command/build/lib/rtps_command/callback_scenario.py\
    aware4docker/qtr-px4-ros2-docker-foxy:1.6.0
    bash