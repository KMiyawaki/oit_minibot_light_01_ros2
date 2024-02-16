#!/bin/bash

function main(){
    local -r PACKAGE="oit_minibot_light_01_ros2"
    local -r DIST=`${HOME}/ros2_ws/src/${PACKAGE}/get_suitable_ros2.sh`
    source /opt/ros/${DIST}/setup.bash
    source ${HOME}/ros2_ws/install/setup.bash
    export ROS_LOCALHOST_ONLY=1
    export ROS_DOMAIN_ID=0
    ros2 run oit_minibot_light_01_ros2 capture --ros-args -p save_dir:="${HOME}/ros2_ws"
}

main "$@"
