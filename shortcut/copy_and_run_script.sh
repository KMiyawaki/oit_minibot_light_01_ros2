#!/bin/bash

function main(){
    if [ $# -ne 1 ];then
        echo "Usage: $0 [script]";
        exit 1
    fi
    local -r DST=${HOME}/ros2_ws/src/`basename ${1}`
    echo "copy from ${1} to ${DST}"
    cp ${1} ${DST}
    chmod u+x ${DST}

    local -r PACKAGE="oit_minibot_light_01_ros2"
    local -r CMD="~/ros2_ws/src/${PACKAGE}/shortcut/docker.sh run_script.sh ${DST}"
    # echo ${CMD}
    eval ${CMD}
}

main "$@"
