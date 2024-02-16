#!/bin/bash

function main(){
    if [ $# -ne 1 ];then
        echo "Usage: $0 [launcher]";
        exit 1
    fi
    local -r PACKAGE="oit_minibot_light_01_ros2"
    ${HOME}/docker_ros_humble/run_prog.sh ${HOME}/ros2_ws/src/${PACKAGE}/launcher/${1}
}

main "$@"
