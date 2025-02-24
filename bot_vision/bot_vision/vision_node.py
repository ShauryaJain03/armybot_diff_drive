#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO

class QRFollower(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        model_path = "/home/shaurya/armybot_ws/src/bot_vision/bot_vision/best.pt"
        self.model = YOLO(model_path)
        
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1
        )
        self.image_publisher = self.create_publisher(Image, '/qr_detected_image', 1)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.confidence_threshold = 0.7
        self.image_center_x = None
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        self.target_area_percent = 20.0 
        
        self.frame_count = 0  # Process every 3rd frame
        self.previous_results = []  # Store last valid results
        self.get_logger().info("QR Follower Node has started")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        if self.image_center_x is None:
            height, width = cv_image.shape[:2]
            self.image_center_x = width // 2
            self.image_width = width
            self.image_height = height
            self.get_logger().info(f"Image dimensions: {width}x{height}")

        if self.frame_count % 3 == 0:
            results = self.model(cv_image)
            self.previous_results = results  # Store valid detection
        else:
            results = self.previous_results  # Use last results

        self.frame_count += 1
        
        vel_cmd = Twist()
        qr_detected = False
        max_conf = 0.0
        qr_center_x = 0
        qr_area_percent = 0
        
        
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                
                if conf > max_conf:
                    max_conf = conf
                    qr_center_x = (x1 + x2) // 2
                    qr_area = (x2 - x1) * (y2 - y1)
                    qr_area_percent = (qr_area / (self.image_width * self.image_height)) * 100
                
                label = f"QR code {conf:.2f}"
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cv2.circle(cv_image, (qr_center_x, (y1 + y2) // 2), 5, (0, 0, 255), -1)
        
        cv2.line(cv_image, (self.image_center_x, 0), (self.image_center_x, self.image_height), (255, 0, 0), 1)
        
        if max_conf > self.confidence_threshold:
            qr_detected = True
            
            x_error = qr_center_x - self.image_center_x
            normalized_error = x_error / (self.image_width / 2)
            
            self.get_logger().info(f"QR detected: conf={max_conf:.2f}, x_error={x_error}, area={qr_area_percent:.1f}%")
            

        
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_publisher.publish(img_msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = QRFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()