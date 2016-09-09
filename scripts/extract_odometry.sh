#!/bin/bash

if [ "$#" -eq 1 ]; then
    rostopic echo -p -b "$1" /odom/pose | cut -d ',' -f 2,3,7 | while read line; do echo "[$line]"; done
    exit 0
fi

if [ "$#" -eq 2 ]; then
    (rostopic echo -p -b "$1" /odom/pose | cut -d ',' -f 2,3,7 | while read line; do echo "[$line]"; done) > "$2"
    exit 0
fi

echo "extract_odometry.sh <ROS bag file> [output file]"
exit 1
