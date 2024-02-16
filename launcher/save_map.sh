#!/bin/bash

function main(){
    local MAP_NAME="test"
    if [ $# -ne 0 ];then
        MAP_NAME=${1}
    fi
    readonly ${MAP_NAME}
    local -r PACKAGE="oit_minibot_light_01_ros2"
    local -r DIST=`${HOME}/ros2_ws/src/${PACKAGE}/get_suitable_ros2.sh`
    source /opt/ros/${DIST}/setup.bash
    source ${HOME}/ros2_ws/install/setup.bash
    export ROS_LOCALHOST_ONLY=1
    export ROS_DOMAIN_ID=0
    local -r MAP_DIR=${HOME}/ros2_ws/src/${PACKAGE}/maps
    cd ${MAP_DIR}
    echo "save map to ${MAP_DIR}/${MAP_NAME}"
    ros2 run nav2_map_server map_saver_cli -f ${MAP_NAME}
    cd ${HOME}/ros2_ws/src/${PACKAGE}
    ./build.sh
}

main "$@"
