#!/bin/bash

function main(){
    ./install_ydlidar.sh
    local -r TARGET_ROS=`./get_suitable_ros2.sh`
    local -r ROS_SETUP="/opt/ros/${TARGET_ROS}/setup.bash"
    local -r WS_SETUP="${HOME}/ros2_ws/install/setup.bash"
    source ${ROS_SETUP}
    source ${WS_SETUP}

    cd ${HOME}/ros2_ws/src
    # git clone https://github.com/KMiyawaki/oit_roboclaw_driver2.git
    sudo apt-get install -y ros-${TARGET_ROS}-teleop-twist-keyboard
    sudo apt-get install -y ros-${TARGET_ROS}-key-teleop
    sudo apt-get install -y ros-${TARGET_ROS}-cartographer-ros
    sudo apt-get install -y xterm
        
    sudo /usr/bin/python3 -m pip install pyserial
    cd ${HOME}/ros2_ws/src/oit_minibot_light_01_ros2
    ./build.sh -a
}

main "$@"
