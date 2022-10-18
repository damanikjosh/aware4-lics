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
    -v $(pwd)/scenarios/:/root/scenarios\
    -v $(pwd)/config/illegal_fishing:/root/illegal_fishing\
    -v $(pwd)/test.yaml:/root/PX4-Autopilot/msg/tools/urtps_bridge_topics.yaml\
    -v $(pwd)/startup.sh:/root/startup.sh\
    joshuajdmk/aware4-lics:v1.4\
    bash -c "cd /root/PX4-Autopilot/Tools && ./qtr_multiple_run.sh -f /root/illegal_fishing -w smooth & MicroXRCEAgent udp4 -p 5000 & (sleep 12 && bash)"