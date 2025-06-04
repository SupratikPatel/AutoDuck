#!/usr/bin/env python3

import rospy
import requests
import json
import time
import base64
import os
from io import BytesIO
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# Configuration - Now using ROS parameters and environment variables
NODE_NAME = 'robot_vlm_client_node'

class RobotVLMClient:
    def __init__(self):
        rospy.init_node(NODE_NAME, anonymous=True)
        self.bridge = CvBridge()
        self.current_mode = "map"  # Default to map mode
        self.last_image_sent_time = 0
        self.send_interval = rospy.get_param('~send_interval', 1.0)  # Seconds, corresponds to 1 FPS
        
        # Get configuration from ROS parameters
        self.laptop_ip = rospy.get_param('~laptop_ip', 'localhost')
        self.laptop_port = rospy.get_param('~laptop_port', 5000)
        self.veh_name = rospy.get_param('~veh', os.environ.get('VEHICLE_NAME', 'duckiebot'))
        
        # New parameters inspired by the video
        self.fast_mode = rospy.get_param('~fast_mode', True)  # Enable fast mode by default
        self.current_mission = rospy.get_param('~mission', 'explore')  # Default mission
        self.adaptive_interval = rospy.get_param('~adaptive_interval', True)  # Adapt send interval based on performance
        
        # Configure URLs and topics
        self.server_url = f"http://{self.laptop_ip}:{self.laptop_port}/process_image"
        self.mission_url = f"http://{self.laptop_ip}:{self.laptop_port}/set_mission"
        self.performance_url = f"http://{self.laptop_ip}:{self.laptop_port}/performance"
        self.image_topic = f"/{self.veh_name}/camera_node/image/compressed"
        self.motor_topic = f"/{self.veh_name}/joy_mapper_node/car_cmd"
        self.mode_topic = f"/{self.veh_name}/operation_mode"
        
        # Image processing parameters
        self.image_resolution = rospy.get_param('~image_resolution', [320, 240])  # [W, H]
        self.image_quality = rospy.get_param('~image_quality', 85)  # JPEG quality
        
        # Motion parameters
        self.base_linear_speed = rospy.get_param('~base_linear_speed', 0.2)
        self.base_angular_speed = rospy.get_param('~base_angular_speed', 0.8)
        
        # Performance tracking
        self.performance_stats = {
            'total_requests': 0,
            'successful_requests': 0,
            'average_response_time': 0.0,
            'last_fps': 0.0
        }

        # Motor command publisher
        self.motor_pub = rospy.Publisher(self.motor_topic, Twist2DStamped, queue_size=1)
        
        # Subscriber for camera images (Compressed)
        self.image_sub = rospy.Subscriber(
            self.image_topic, 
            CompressedImage, 
            self.image_callback_compressed, 
            queue_size=1, 
            buff_size=2**24
        )

        # Subscriber for operation mode
        rospy.Subscriber(self.mode_topic, String, self.operation_mode_callback)
        
        # Subscriber for mission changes (new feature)
        rospy.Subscriber(f"/{self.veh_name}/vlm_mission", String, self.mission_callback)

        # Initialize mission on server
        self.set_mission(self.current_mission)

        rospy.loginfo(f"{NODE_NAME} started successfully")
        rospy.loginfo(f"Vehicle: {self.veh_name}")
        rospy.loginfo(f"VLM Server: {self.server_url}")
        rospy.loginfo(f"Fast Mode: {'ENABLED' if self.fast_mode else 'DISABLED'}")
        rospy.loginfo(f"Mission: {self.current_mission}")
        rospy.loginfo(f"Image topic: {self.image_topic}")
        rospy.loginfo(f"Motor topic: {self.motor_topic}")
        rospy.loginfo(f"Mode topic: {self.mode_topic}")
        rospy.loginfo(f"Current mode: {self.current_mode}")

    def operation_mode_callback(self, msg):
        if msg.data != self.current_mode:
            self.current_mode = msg.data
            rospy.loginfo(f"Operation mode changed to: {self.current_mode}")
            if self.current_mode != "vlm":
                # Stop motors if switching out of VLM mode
                self.send_motor_command(0.0, 0.0)

    def mission_callback(self, msg):
        """Handle mission changes from ROS topic"""
        new_mission = msg.data.lower()
        if new_mission != self.current_mission:
            self.current_mission = new_mission
            self.set_mission(new_mission)
            rospy.loginfo(f"Mission changed to: {new_mission}")

    def set_mission(self, mission):
        """Set mission on VLM server"""
        try:
            response = requests.post(self.mission_url, params={"mission": mission}, timeout=5)
            if response.status_code == 200:
                rospy.loginfo(f"Mission '{mission}' set successfully on VLM server")
            else:
                rospy.logwarn(f"Failed to set mission on server: {response.status_code}")
        except Exception as e:
            rospy.logwarn(f"Could not set mission on server: {e}")

    def get_performance_stats(self):
        """Get performance statistics from VLM server"""
        try:
            response = requests.get(self.performance_url, timeout=2)
            if response.status_code == 200:
                return response.json()
        except:
            pass
        return None

    def update_adaptive_interval(self, processing_time):
        """Dynamically adjust send interval based on performance (inspired by video)"""
        if not self.adaptive_interval:
            return
            
        # Target: maintain >1 FPS as shown in the video
        target_fps = 1.5 if self.fast_mode else 1.0
        target_interval = 1.0 / target_fps
        
        # Add some buffer for network latency
        optimal_interval = max(processing_time + 0.2, target_interval)
        
        # Smooth adjustment
        self.send_interval = 0.7 * self.send_interval + 0.3 * optimal_interval
        self.send_interval = max(0.5, min(self.send_interval, 3.0))  # Clamp between 0.5-3 seconds

    def image_callback_compressed(self, ros_image_compressed):
        if self.current_mode != "vlm":
            return

        current_time = time.time()
        if current_time - self.last_image_sent_time < self.send_interval:
            return  # Limit frame rate
        self.last_image_sent_time = current_time

        rospy.logdebug("Processing compressed image for VLM")
        try:
            # Fixed image decoding for Python 3 and modern numpy
            np_arr = np.frombuffer(ros_image_compressed.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logerr("Failed to decode compressed image")
                return
                
        except Exception as e:
            rospy.logerr(f"Error decoding compressed image: {e}")
            return

        self.process_and_send_image(cv_image)

    def process_and_send_image(self, cv_image):
        # Resize image
        resized_image = cv2.resize(cv_image, tuple(self.image_resolution))

        # Encode to JPEG
        try:
            success, buffer = cv2.imencode(
                ".jpg", 
                resized_image, 
                [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]
            )
            if not success:
                rospy.logwarn("Could not encode image to JPEG")
                return
            image_bytes = buffer.tobytes()
            image_base64 = base64.b64encode(image_bytes).decode('utf-8')
        except Exception as e:
            rospy.logerr(f"Error encoding image to JPEG/Base64: {e}")
            return

        # Enhanced prompt based on current mission
        mission_prompts = {
            'explore': "Explore the environment safely, avoiding obstacles and looking for interesting areas.",
            'find_books': "Look for books or text objects in the environment. Move towards any books you see.",
            'find_friends': "Search for other robots or friendly objects. Move towards potential robot friends.",
            'navigate': "Navigate through the space efficiently while avoiding obstacles.",
            'clean': "Look for areas that need cleaning or organizing. Move towards cluttered areas."
        }
        
        base_prompt = mission_prompts.get(self.current_mission, mission_prompts['explore'])
        
        if self.fast_mode:
            prompt = f"Mission: {self.current_mission}. {base_prompt} Respond with only: FORWARD, LEFT, RIGHT, or STOP."
        else:
            prompt = f"Mission: {self.current_mission}. {base_prompt} Explain your reasoning briefly."
        
        payload = {
            "image_base64": image_base64, 
            "prompt": prompt,
            "fast_mode": self.fast_mode  # New parameter
        }
        
        request_start_time = time.time()
        self.performance_stats['total_requests'] += 1
        
        try:
            rospy.logdebug(f"Sending image to VLM server (Fast Mode: {self.fast_mode})...")
            response = requests.post(
                self.server_url, 
                json=payload, 
                timeout=15  # Increased timeout for VLM processing
            )
            response.raise_for_status()
            
            request_time = time.time() - request_start_time
            command_data = response.json()
            
            # Update performance statistics
            self.performance_stats['successful_requests'] += 1
            self.performance_stats['average_response_time'] = (
                0.8 * self.performance_stats['average_response_time'] + 
                0.2 * request_time
            )
            self.performance_stats['last_fps'] = 1.0 / request_time if request_time > 0 else 0
            
            # Update adaptive interval based on actual processing time
            if 'processing_time' in command_data:
                self.update_adaptive_interval(command_data['processing_time'])
            
            # Log performance info
            fps = self.performance_stats['last_fps']
            rospy.loginfo(f"VLM Response: {command_data.get('action', 'unknown').upper()} "
                         f"(FPS: {fps:.2f}, Time: {request_time:.2f}s)")
            
            if 'reasoning' in command_data and not self.fast_mode:
                rospy.loginfo(f"Reasoning: {command_data['reasoning']}")
            
            self.execute_command(command_data)

        except requests.exceptions.Timeout:
            rospy.logwarn("VLM server timeout - stopping for safety")
            self.send_motor_command(0.0, 0.0)
        except requests.exceptions.ConnectionError:
            rospy.logwarn("Cannot connect to VLM server - stopping for safety")
            self.send_motor_command(0.0, 0.0)
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"HTTP request failed: {e}")
            self.send_motor_command(0.0, 0.0)
        except ValueError as e:
            rospy.logerr(f"Could not decode JSON response: {e}")
            self.send_motor_command(0.0, 0.0)
        except Exception as e:
            rospy.logerr(f"Unexpected error during VLM communication: {e}")
            self.send_motor_command(0.0, 0.0)

    def execute_command(self, command_data):
        if self.current_mode != "vlm":
            return

        action = command_data.get("action", "stop").lower()
        speed_multiplier = command_data.get("speed", 0.5)

        linear_x = 0.0
        angular_z = 0.0

        if action == "forward":
            linear_x = self.base_linear_speed * speed_multiplier
        elif action == "left":
            angular_z = self.base_angular_speed * speed_multiplier
        elif action == "right":
            angular_z = -self.base_angular_speed * speed_multiplier
        elif action == "stop":
            linear_x = 0.0
            angular_z = 0.0
        else:
            rospy.logwarn(f"Unknown action: {action} - stopping for safety")
            linear_x = 0.0
            angular_z = 0.0

        self.send_motor_command(linear_x, angular_z)

    def send_motor_command(self, linear_x, angular_z):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = linear_x  # Linear velocity
        msg.omega = angular_z  # Angular velocity
        
        self.motor_pub.publish(msg)
        if linear_x != 0 or angular_z != 0:
            rospy.logdebug(f"Motor command: v={linear_x:.2f}, omega={angular_z:.2f}")

    def print_performance_stats(self):
        """Print performance statistics periodically"""
        if self.performance_stats['total_requests'] > 0:
            success_rate = (self.performance_stats['successful_requests'] / 
                          self.performance_stats['total_requests']) * 100
            
            rospy.loginfo(f"Performance Stats - "
                         f"Requests: {self.performance_stats['total_requests']}, "
                         f"Success Rate: {success_rate:.1f}%, "
                         f"Avg Response: {self.performance_stats['average_response_time']:.2f}s, "
                         f"Current FPS: {self.performance_stats['last_fps']:.2f}, "
                         f"Send Interval: {self.send_interval:.2f}s")

    def run(self):
        rospy.loginfo("VLM Client ready and waiting for mode switch to 'vlm'")
        rospy.loginfo(f"Performance Mode: {'FAST' if self.fast_mode else 'DETAILED'}")
        
        # Print performance stats every 30 seconds
        last_stats_time = time.time()
        
        rate = rospy.Rate(1)  # 1 Hz for stats checking
        while not rospy.is_shutdown():
            current_time = time.time()
            if current_time - last_stats_time > 30:  # Every 30 seconds
                self.print_performance_stats()
                last_stats_time = current_time
            
            rate.sleep()

if __name__ == '__main__':
    try:
        client = RobotVLMClient()
        client.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("VLM Client node interrupted")
    except Exception as e:
        rospy.logerr(f"VLM Client failed to start: {e}")
    finally:
        rospy.loginfo("VLM Client shutting down")
