#!/bin/bash

function main(){
    cd "$(dirname "$0")"
    local -r VERSION_ID=`./get_ubuntu_version.sh`
    if [[ "${VERSION_ID}" =~ "24" ]]; then
        echo "jazzy"
    elif [[ "${VERSION_ID}" =~ "22" ]]; then
        echo "humble"
    elif [[ "${VERSION_ID}" =~ "20" ]]; then
        echo "foxy"
    else
        echo "*** No supported ROS2 found ***"
    fi
}

main "$@"
