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
        
        # Auto-detect VLM implementation
        self.vlm_implementation = self.detect_vlm_implementation()
        rospy.loginfo(f"Detected VLM implementation: {self.vlm_implementation}")
        
        # Configure URLs and topics based on implementation
        if self.vlm_implementation == "llamacpp":
            # Use new llama.cpp API endpoints
            self.server_url = f"http://{self.laptop_ip}:{self.laptop_port}/api/analyze"
            self.stats_url = f"http://{self.laptop_ip}:{self.laptop_port}/api/stats"
            self.health_url = f"http://{self.laptop_ip}:{self.laptop_port}/api/health"
        else:
            # Use legacy Ollama endpoints
            self.server_url = f"http://{self.laptop_ip}:{self.laptop_port}/process_image"
            self.stats_url = f"http://{self.laptop_ip}:{self.laptop_port}/performance"
            self.health_url = f"http://{self.laptop_ip}:{self.laptop_port}/status"
        
        self.image_topic = f"/{self.veh_name}/camera_node/image/compressed"
        self.motor_topic = f"/{self.veh_name}/joy_mapper_node/car_cmd"
        self.mode_topic = f"/{self.veh_name}/operation_mode"
        
        # Image processing parameters
        self.image_resolution = rospy.get_param('~image_resolution', [640, 480])  # Updated for better performance
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

        # Wait for VLM server to be ready
        self.wait_for_server()

        rospy.loginfo(f"{NODE_NAME} started successfully")
        rospy.loginfo(f"Vehicle: {self.veh_name}")
        rospy.loginfo(f"VLM Server: {self.server_url}")
        rospy.loginfo(f"Implementation: {self.vlm_implementation}")
        rospy.loginfo(f"Fast Mode: {'ENABLED' if self.fast_mode else 'DISABLED'}")
        rospy.loginfo(f"Mission: {self.current_mission}")
        rospy.loginfo(f"Image topic: {self.image_topic}")
        rospy.loginfo(f"Motor topic: {self.motor_topic}")
        rospy.loginfo(f"Mode topic: {self.mode_topic}")
        rospy.loginfo(f"Current mode: {self.current_mode}")

    def detect_vlm_implementation(self):
        """Auto-detect which VLM implementation is running"""
        try:
            # Try llamacpp endpoints first
            response = requests.get(f"http://{self.laptop_ip}:{self.laptop_port}/api/health", timeout=5)
            if response.status_code == 200:
                data = response.json()
                if 'llama.cpp' in str(data).lower() or 'llamacpp' in str(data).lower():
                    return "llamacpp"
        except:
            pass
            
        try:
            # Try ollama endpoints
            response = requests.get(f"http://{self.laptop_ip}:{self.laptop_port}/status", timeout=5)
            if response.status_code == 200:
                return "ollama"
        except:
            pass
            
        # Default to ollama for backward compatibility
        return "ollama"

    def wait_for_server(self):
        """Wait for VLM server to be ready"""
        rospy.loginfo("Waiting for VLM server to be ready...")
        max_attempts = 30
        for attempt in range(max_attempts):
            try:
                response = requests.get(self.health_url, timeout=2)
                if response.status_code == 200:
                    rospy.loginfo("✅ VLM server is ready!")
                    return
            except:
                pass
            rospy.loginfo(f"⏳ VLM server not ready, attempt {attempt + 1}/{max_attempts}")
            time.sleep(2)
        
        rospy.logwarn("⚠️ VLM server may not be ready, proceeding anyway...")

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
            rospy.loginfo(f"Mission changed to: {new_mission}")

    def get_performance_stats(self):
        """Get performance statistics from VLM server"""
        try:
            response = requests.get(self.stats_url, timeout=2)
            if response.status_code == 200:
                return response.json()
        except:
            pass
        return None

    def update_adaptive_interval(self, processing_time):
        """Dynamically adjust send interval based on performance"""
        if not self.adaptive_interval:
            return
            
        # For llamacpp: target higher FPS due to better performance
        if self.vlm_implementation == "llamacpp":
            target_fps = 0.8  # ~1.2s intervals for real-time control
        else:
            target_fps = 0.3  # ~3s intervals for ollama
            
        target_interval = 1.0 / target_fps
        
        # Add some buffer for network latency
        optimal_interval = max(processing_time + 0.2, target_interval)
        
        # Smooth adjustment
        self.send_interval = 0.7 * self.send_interval + 0.3 * optimal_interval
        self.send_interval = max(0.5, min(self.send_interval, 5.0))  # Clamp between 0.5-5 seconds

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

        # Create request payload based on implementation
        start_time = time.time()
        
        try:
            if self.vlm_implementation == "llamacpp":
                # New llamacpp format
                payload = {
                    "image_base64": image_base64
                }
                response = requests.post(self.server_url, json=payload, timeout=15)
            else:
                # Legacy ollama format  
                payload = {
                    "image_base64": image_base64,
                    "fast_mode": self.fast_mode
                }
                response = requests.post(self.server_url, json=payload, timeout=15)
            
            processing_time = time.time() - start_time
            
            if response.status_code == 200:
                self.performance_stats['total_requests'] += 1
                self.performance_stats['successful_requests'] += 1
                
                # Update adaptive interval
                self.update_adaptive_interval(processing_time)
                
                # Parse response based on implementation
                result = response.json()
                if self.vlm_implementation == "llamacpp":
                    command_data = {
                        'action': result.get('decision', 'stop').lower(),
                        'reasoning': result.get('reasoning', ''),
                        'processing_time': processing_time
                    }
                else:
                    command_data = {
                        'action': result.get('action', 'stop'),
                        'reasoning': result.get('reasoning', ''),  
                        'processing_time': processing_time
                    }
                
                # Execute the command
                self.execute_command(command_data)
                
                rospy.loginfo(f"🚗 {command_data['action'].upper()} | {processing_time:.2f}s | {command_data['reasoning'][:50]}...")
                
            else:
                rospy.logwarn(f"VLM server responded with status {response.status_code}")
                self.performance_stats['total_requests'] += 1
                
        except Exception as e:
            rospy.logerr(f"Error communicating with VLM server: {e}")
            self.performance_stats['total_requests'] += 1
            # Default to safe action
            self.send_motor_command(0.0, 0.0)

    def execute_command(self, command_data):
        """Execute driving command based on VLM decision"""
        action = command_data['action']
        
        if action == "forward":
            linear_x = self.base_linear_speed
            angular_z = 0.0
        elif action == "left":
            linear_x = self.base_linear_speed * 0.5
            angular_z = self.base_angular_speed
        elif action == "right":
            linear_x = self.base_linear_speed * 0.5
            angular_z = -self.base_angular_speed
        elif action == "stop":
            linear_x = 0.0
            angular_z = 0.0
        else:
            # Unknown command, default to stop
            rospy.logwarn(f"Unknown action '{action}', defaulting to stop")
            linear_x = 0.0
            angular_z = 0.0
        
        self.send_motor_command(linear_x, angular_z)

    def send_motor_command(self, linear_x, angular_z):
        """Send motor command to robot"""
        cmd = Twist2DStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.v = linear_x
        cmd.omega = angular_z
        self.motor_pub.publish(cmd)

    def print_performance_stats(self):
        """Print performance statistics"""
        if self.performance_stats['total_requests'] > 0:
            success_rate = (self.performance_stats['successful_requests'] / 
                          self.performance_stats['total_requests']) * 100
            rospy.loginfo(f"📊 Performance: {success_rate:.1f}% success rate, "
                         f"{self.performance_stats['total_requests']} total requests")

    def run(self):
        """Main run loop"""
        rate = rospy.Rate(0.1)  # 10 second intervals for stats
        while not rospy.is_shutdown():
            self.print_performance_stats()
            rate.sleep()

if __name__ == '__main__':
    try:
        client = RobotVLMClient()
        client.run()
    except rospy.ROSInterruptException:
        pass
