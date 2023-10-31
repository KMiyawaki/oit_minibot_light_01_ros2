#!/bin/bash
cd $(dirname $0)
rosbags-convert ${1} --include-topic /odom --include-topic /scan --include-topic /tf --include-topic /tf_static
