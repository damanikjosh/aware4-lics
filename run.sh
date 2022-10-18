#!/bin/bash

if [ "$#" -lt 1 ]; then
        echo usage: ./run.sh scenario
        exit 1
fi

xhost +local:docker
(./QGroundControl.AppImage > /dev/null 2>&1) &
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
    --shm-size=16G\
    --network host\
    --name cisim\
    -v $(pwd)/startup.sh:/root/startup.sh\
    -v $(pwd)/$1/:/root/scenario\
    -v $(pwd)/urtps_bridge_topics.yaml:/root/PX4-Autopilot/msg/tools/urtps_bridge_topics.yaml\
    joshuajdmk/aware4-lics:v1.4\
    /root/startup.sh

pkill -9 QGroundControl