#!/bin/bash
function main(){
    if [ $# -eq 0 ];then
        local NODE=$(cd $(dirname $0);pwd)
        NODE=`echo "$NODE" | sed -e 's/.*\/\([^\/]*\)$/\1/'`
        cd ~/ros2_ws && colcon build --symlink-install --packages-select ${NODE}
    else
        cd ~/ros2_ws && colcon build --symlink-install
    fi
}

main "$@"
