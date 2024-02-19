#!/bin/bash

function main(){
    if [ $# -ne 1 ];then
        echo "Usage: $0 [script]";
        exit 1
    fi
    local -r PACKAGE="oit_minibot_light_01_ros2"
    local -r DIST=`${HOME}/ros2_ws/src/${PACKAGE}/get_suitable_ros2.sh`
    source /opt/ros/${DIST}/setup.bash
    source ${HOME}/ros2_ws/install/setup.bash
    export ROS_LOCALHOST_ONLY=1
    export ROS_DOMAIN_ID=0
    python ${1}
}

main "$@"
