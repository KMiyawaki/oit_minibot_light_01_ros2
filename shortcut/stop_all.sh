#!/bin/bash

function main(){
    local -r PACKAGE="oit_minibot_light_01_ros2"
    local -r DIST=`${HOME}/ros2_ws/src/${PACKAGE}/get_suitable_ros2.sh`
    source /opt/ros/${DIST}/setup.bash
    source ${HOME}/ros2_ws/install/setup.bash
    if [[ "${DIST}" =~ "foxy" ]] || [[ "${DIST}" =~ "humble" ]]; then
        export ROS_LOCALHOST_ONLY=1
    else
        export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    fi
    export ROS_DOMAIN_ID=0
    
    # ros2 topic pub -r 2 -t 5 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'

    local PROGS=`ps -aux | grep ros | grep -v grep | awk '{ printf("%d ", $2);for(i=11;i<=NF;++i){ printf("%s ",$i) };printf("\n") }'`
    while read p
    do
        arr=($p)
        cmd="kill -9 ${arr[0]}"
        echo "${cmd} ${arr[1]} ${arr[2]}"
        eval ${cmd}
    done <<< "${PROGS}"
}

main "$@"
