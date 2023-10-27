#!/bin/bash

function main(){
    local -r TARGET_ROS=`./get_suitable_ros2.sh`
    local -r ROS_SETUP="/opt/ros/${TARGET_ROS}/setup.bash"
    local -r WS_SETUP="${HOME}/ros2_ws/install/setup.bash"
    source ${ROS_SETUP}
    source ${WS_SETUP}
    
    cd
    git clone https://github.com/YDLIDAR/YDLidar-SDK.git
    mkdir -p YDLidar-SDK/build
    cd YDLidar-SDK/build
    cmake ..
    make
    sudo make install
    
    cd ${HOME}/ros2_ws/src
    if [[ "${TARGET_ROS}" =~ "humble" ]]; then
        git clone https://github.com/YDLIDAR/ydlidar_ros2_driver -b humble
    else
        git clone https://github.com/YDLIDAR/ydlidar_ros2_driver
    fi
    cd ${HOME}/ros2_ws
    colcon build --symlink-install
    
    echo "Run the following commands"
    echo "cd ~/ros2_ws/src/ydlidar_ros2_driver/startup"
    echo "sudo sh initenv.sh"
}

main "$@"
