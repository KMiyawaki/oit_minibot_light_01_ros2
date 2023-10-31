#!/bin/bash -x
function main(){
    cd $(dirname $0)
    ros2 launch oit_minibot_light_01_ros2 play_rosbag.launch.py bag:=${1} ${2}
}

main "$@"
