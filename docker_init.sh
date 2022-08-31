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
    -v $(pwd)/models/cylinder_area:/root/PX4-Autopilot/Tools/sitl_gazebo/models/cylinder_area\
    -v $(pwd)/models/rect_area:/root/PX4-Autopilot/Tools/sitl_gazebo/models/rect_area\
    -v $(pwd)/worlds/muin_area.world:/root/PX4-Autopilot/Tools/sitl_gazebo/worlds/muin_area.world\
    -v $(pwd)/scenarios/test_scenario.py:/root/px4_ros_com_ros2/build/rtps_command/build/lib/rtps_command/test_scenario.py\
    aware4docker/qtr-px4-ros2-docker-foxy:1.5.6 
    bash