#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import math

class QRFollowerWithObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('vision_with_obstacle_avoidance')
        
        model_path = "/home/shaurya/armybot_ws/src/bot_vision/bot_vision/best.pt"
        self.model = YOLO(model_path)
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            1
        )
        
        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            "/scan", 
            self.scan_callback, 
            40
        )
        
        self.image_publisher = self.create_publisher(Image, '/qr_detected_image', 1)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/bot_controller/cmd_vel_unstamped', 10)
        
        self.confidence_threshold = 0.7
        self.image_center_x = None
        self.max_linear_speed = 0.5
        self.max_angular_speed = 0.5
        self.target_area_percent = 12 
        
        self.frame_count = 0 
        self.previous_results = []
        
        self.regions = {'right': 100, 'mid': 100, 'left': 100}
        self.safe_distance = 1.5
        
  
        self.obstacle_state = {
            'detected': False,
            'turn_direction': 0,  
            'turn_start_time': None,
            'turn_duration': 0.8  
        }
        
     
        self.control_mode = "IDLE" 
        
        self.get_logger().info("QR Follower with Improved Obstacle Avoidance Node has started")

    def scan_callback(self, scan_data):
        # Process LiDAR scan data
        self.regions = {
            'left':   min(min(scan_data.ranges[0:60]), 100),
            'mid':    min(min(scan_data.ranges[60:300]), 100),
            'right':  min(min(scan_data.ranges[300:360]), 100),
        }
        
        if (self.regions['right'] < self.safe_distance or 
            self.regions['mid'] < self.safe_distance or 
            self.regions['left'] < self.safe_distance):
            
            if not self.obstacle_state['detected']:
                if self.regions['left'] < self.safe_distance:
                    self.obstacle_state['turn_direction'] = -1  
                    print("Turn right")
                elif self.regions['right'] < self.safe_distance:
                    self.obstacle_state['turn_direction'] = 1  
                    print("Turn left")

                else:
                    self.obstacle_state['turn_direction'] = 1  
                    print("mid blocked")
                
                self.obstacle_state['turn_start_time'] = self.get_clock().now().seconds_nanoseconds()[0]
                self.obstacle_state['detected'] = True
                self.control_mode = "OBSTACLE_AVOIDANCE"
            
            self.get_logger().info(f"Obstacle detected: Turn Direction = {self.obstacle_state['turn_direction']}")
        else:
            if self.obstacle_state['detected']:
                self.obstacle_state['detected'] = False
                self.obstacle_state['turn_direction'] = 0
                self.obstacle_state['turn_start_time'] = None
                
                self.control_mode = "IDLE"
        
        self.send_velocity_commands()

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
            self.previous_results = results  
        else:
            results = self.previous_results  

        self.frame_count += 1
        
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
            self.qr_center_x = qr_center_x
            self.qr_area_percent = qr_area_percent
            
            if not self.obstacle_state['detected']:
                self.control_mode = "QR_FOLLOWING"
                
            self.get_logger().info(f"QR detected: conf={max_conf:.2f}, area={qr_area_percent:.1f}%, mode={self.control_mode}")
        else:
            if self.control_mode == "QR_FOLLOWING":
                self.control_mode = "IDLE"
        
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_publisher.publish(img_msg)
        
        self.send_velocity_commands()

    def send_velocity_commands(self):
        vel_cmd = Twist()
        
        if self.control_mode == "OBSTACLE_AVOIDANCE":
            if self.obstacle_state['turn_start_time'] is not None:
                current_time = self.get_clock().now().seconds_nanoseconds()[0]
                elapsed_time = current_time - self.obstacle_state['turn_start_time']
                
                if elapsed_time < self.obstacle_state['turn_duration']:
                    vel_cmd.angular.z = 0.5 * self.obstacle_state['turn_direction']
                    vel_cmd.linear.x = 0.8
                    self.get_logger().info(f"Turning to avoid obstacle: direction={self.obstacle_state['turn_direction']}")
                else:
                    self.obstacle_state['detected'] = False
                    self.obstacle_state['turn_direction'] = 0
                    self.obstacle_state['turn_start_time'] = None
                    self.control_mode = "IDLE"
            
        elif self.control_mode == "QR_FOLLOWING":
            x_error = self.qr_center_x - self.image_center_x
            normalized_error = x_error / (self.image_width / 2)
            
            if self.qr_area_percent > self.target_area_percent:
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = 0.0
                self.get_logger().info("Target reached, stopping")
            else:
                vel_cmd.linear.x = self.max_linear_speed
                vel_cmd.angular.z = -normalized_error * self.max_angular_speed
                self.get_logger().info(f"Following QR: linear={vel_cmd.linear.x:.2f}, angular={vel_cmd.angular.z:.2f}")
        
        else:  
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(vel_cmd)
        self.get_logger().debug(f"Mode: {self.control_mode}, Sent vel cmd: linear={vel_cmd.linear.x:.2f}, angular={vel_cmd.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = QRFollowerWithObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()