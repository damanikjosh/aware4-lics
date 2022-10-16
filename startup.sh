#!/bin/bash

cd /root/PX4-Autopilot && make px4_sitl_rtps && Tools/qtr_multiple_run.sh -s QTR:1 -w smooth && tail -f /dev/null