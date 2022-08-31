#!/bin/bash

if [ "$#" -lt 7 ]; then
        echo usage: test_process.sh px4_build_path ros2_build_path scenario_name headless_flag model_name world_name num_vehicles
        exit 1
fi

px4_build_path="$1"
ros2_build_path="$2"
scenario_name="$3"
headless_flag="$4"
model_name="$5"
world_name="$6"
num_vehicles="$7"

echo px4_build_path: $px4_build_path
echo ros2_build_path: $ros2_build_path
echo scenario_name: $scenario_name
echo headless_flag: $headless_flag
echo model_name: $model_name
echo world_name: $world_name
echo num_vehicles: $num_vehicles

function cleanup() {
        pkill --echo px4
        pkill gzclient
        pkill gzserver
        pkill -x micrortps_agent
        ros2 daemon stop &&
        ros2 daemon start
}


source /opt/ros/foxy/setup.bash
export CMAKE_PREFIX_PATH=/opt/ros/foxy
source /root/px4_ros_com_ros2/install/setup.bash

ret=1

echo "[Starting PX4 instance]"
cd $px4_build_path/Tools
./qtr_multiple_run.sh -s $model_name -q $headless_flag -w $world_name &&

if [ $? -eq 0 ]
then
        echo "[Run MircoRTPS]"
        cd /root
        ./microRTPS_multiple_agent_run.sh --kill &
        sleep 1
        ./microRTPS_multiple_agent_run.sh -n $num_vehicles &&

        if [ $? -eq 0 ]
        then
                echo "[Run Scenario]"
                cd $ros2_build_path
                python3 build/rtps_command/build/lib/rtps_command/${scenario_name}
                ret=$?
                echo $ret
        else
                echo "[Failed to start micro-agent]"
        fi

else
        echo "[Failed to start px4]"
        exit 1

fi

trap "cleanup" SIGINT SIGTERM EXIT

if [ $ret -eq 0 ]
then
        echo "pass"
        exit 0
else
        echo "fail"
        exit 1
fi