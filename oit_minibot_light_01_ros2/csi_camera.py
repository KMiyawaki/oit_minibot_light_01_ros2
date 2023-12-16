#!/usr/bin/env python
# -*- coding: utf_8 -*-

import os
import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

class CSICameraNode(Node):
    def __init__(self, node_name, fps = 30):
        super().__init__(node_name)
        self.pub_image = self.create_publisher(Image, '/video_source/raw', 10)
        self.bridge = cv_bridge.CvBridge()
        self.gstreamer_pipline = gstreamer_pipeline(framerate=fps)
        self.video_capture = cv2.VideoCapture(self.gstreamer_pipline, cv2.CAP_GSTREAMER)
        self.timer_period = 1.0 / float(fps)  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    def timer_callback(self):
        ret_val, frame = self.video_capture.read()
        if ret_val == False or frame is None:
            self.get_logger().error('self.video_capture.read() Failed')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub_image.publish(msg)

def main(args=None):
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    try:
        rclpy.init(args=args)
        node = CSICameraNode(node_name)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Exiting %s" % node.get_name())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
