#!/usr/bin/env python
# -*- coding: utf_8 -*-

import datetime
import os
import cv2
import cv_bridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageCaptureNode(Node):
    def __init__(self, node_name, fps = 30):
        super().__init__(node_name)
        self.save_dir = self.declare_parameter('save_dir', '.').get_parameter_value().string_value
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image,'/video_source/raw',self.image_cb,10)
    
    def image_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            d = datetime.datetime.now()
            file_name = self.save_dir + '/' + d.strftime('%Y%m%d_%H%M%S.jpg')
            cv2.imwrite(file_name, cv_image)
            self.get_logger().info('Saved %s' % file_name)
            raise SystemExit
        except cv_bridge.CvBridgeError as e:
            print(e)

def main(args=None):
    script_name = os.path.basename(__file__)
    node_name = os.path.splitext(script_name)[0]
    try:
        rclpy.init(args=args)
        node = ImageCaptureNode(node_name)
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.get_logger().info("Exiting %s" % node.get_name())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
