#!/usr/bin/env python3
"""
Duckiebot VLM Navigation System
Integrates Vision-Language Model with direct robot control for autonomous navigation

This system:
1. Gets video feed directly from DuckieBot's ROS camera topic
2. Analyzes frames using Qwen2.5-VL for navigation decisions
3. Sends direct ROS control commands to the robot (no keyboard interface)
4. Provides lane following and obstacle avoidance capabilities

Usage:
    python duckiebot_vlm_navigation.py <robot_name> [--llama-url <url>] [--fps <fps>]
"""

import rospy
import cv2
import numpy as np
import base64
import requests
import json
import time
import threading
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Float32, Header
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from cv_bridge import CvBridge, CvBridgeError
import argparse
from datetime import datetime
import signal
import sys

class DuckiebotVLMNavigator:
    def __init__(self, robot_name, llama_server_url="http://localhost:8080", processing_fps=2.0):
        self.robot_name = robot_name
        self.llama_server_url = llama_server_url
        self.processing_fps = processing_fps
        self.processing_interval = 1.0 / processing_fps
        self.bridge = CvBridge()
        
        # Processing control
        self.last_process_time = 0
        self.processing = False
        self.auto_control = True
        
        # Performance tracking
        self.stats = {
            'total_frames': 0,
            'processed_frames': 0,
            'successful_decisions': 0,
            'failed_decisions': 0,
            'avg_response_time': 0.0,
            'camera_fps': 0.0,
            'decisions': {'FORWARD': 0, 'LEFT': 0, 'RIGHT': 0, 'STOP': 0}
        }
        
        # Camera FPS tracking
        self.last_frame_time = 0
        self.fps_tracker = []
        
        # Control parameters (from duckie_v2 motor controller)
        self.max_speed = 0.15  # m/s - conservative speed for safety
        self.max_angular = 1.0  # rad/s
        self.wheel_distance = 0.1  # meters between wheels
        
        # Navigation state
        self.current_decision = None
        self.last_decision_time = 0
        self.decision_history = []
        
        # Initialize ROS
        rospy.init_node('duckiebot_vlm_navigator', anonymous=True)
        
        # Publishers for robot control (using duckie_v2 approach)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.wheels_pub = rospy.Publisher(f'/{robot_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.duckiebot_vel_pub = rospy.Publisher(f'/{robot_name}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1)
        
        # Publishers for monitoring and debugging
        from std_msgs.msg import String
        self.decision_pub = rospy.Publisher('/vlm_navigator/current_decision', String, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/vlm_navigator/debug_image', Image, queue_size=1)
        self.performance_pub = rospy.Publisher('/vlm_navigator/performance', Float32, queue_size=1)
        
        # Subscriber to camera feed (using duckie_v2 approach)
        self.image_topic = f"/{robot_name}/camera_node/image/compressed"
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # Emergency stop subscriber
        self.emergency_stop = False
        self.emergency_sub = rospy.Subscriber('/vlm_navigator/emergency_stop', Bool, self.emergency_callback)
        
        rospy.loginfo(f"🤖 VLM Navigator initialized for robot: {robot_name}")
        rospy.loginfo(f"📹 Camera topic: {self.image_topic}")
        rospy.loginfo(f"🧠 VLM server: {llama_server_url}")
        rospy.loginfo(f"⚡ Processing FPS: {processing_fps}")
        
        # Setup graceful shutdown
        signal.signal(signal.SIGINT, self.shutdown_handler)
        
    def emergency_callback(self, msg):
        """Handle emergency stop commands"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            rospy.logwarn("🚨 EMERGENCY STOP ACTIVATED")
            self.stop_robot()
    
    def shutdown_handler(self, signum, frame):
        """Graceful shutdown handler"""
        rospy.loginfo("🛑 Shutting down VLM Navigator...")
        self.auto_control = False
        self.stop_robot()
        rospy.signal_shutdown("User interrupt")
        sys.exit(0)
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Update camera FPS tracking
            self.update_camera_fps()
            self.stats['total_frames'] += 1
            
            # Rate limiting for processing
            current_time = time.time()
            if current_time - self.last_process_time < self.processing_interval:
                return
            
            # Skip if already processing
            if self.processing:
                return
            
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logerr("Failed to decode camera image")
                return
            
            # Process asynchronously to avoid blocking ROS
            self.processing = True
            self.last_process_time = current_time
            
            # Process in separate thread
            thread = threading.Thread(target=self._async_process_frame, args=(cv_image.copy(),))
            thread.daemon = True
            thread.start()
            
        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")
            self.processing = False
    
    def _async_process_frame(self, image):
        """Asynchronously process frame with VLM"""
        try:
            # Resize image for faster processing (optional)
            height, width = image.shape[:2]
            if width > 640:
                scale = 640.0 / width
                new_width = int(width * scale)
                new_height = int(height * scale)
                image = cv2.resize(image, (new_width, new_height))
            
            # Analyze with VLM
            result = self.analyze_image_with_vlm(image)
            
            if result and result['success']:
                decision = result['decision']
                reasoning = result['reasoning']
                response_time = result['response_time']
                
                # Update statistics
                self.update_stats(decision, response_time, success=True)
                
                # Log decision
                rospy.loginfo(f"🧠 {decision} | {response_time:.2f}s | {reasoning[:80]}...")
                
                # Execute control if auto control is enabled
                if self.auto_control and not self.emergency_stop:
                    self.execute_navigation_command(decision)
                
                # Publish decision for monitoring
                self.decision_pub.publish(String(f"{decision}: {reasoning}"))
                
                # Update history
                self.current_decision = decision
                self.last_decision_time = time.time()
                self.decision_history.append({
                    'timestamp': datetime.now().isoformat(),
                    'decision': decision,
                    'reasoning': reasoning,
                    'response_time': response_time
                })
                
                # Keep only last 100 decisions
                if len(self.decision_history) > 100:
                    self.decision_history = self.decision_history[-100:]
            
            else:
                # Handle VLM failure
                self.update_stats(None, 0, success=False)
                if not self.emergency_stop:
                    rospy.logwarn("⚠️ VLM analysis failed, executing safety stop")
                    self.execute_navigation_command('STOP')
            
        except Exception as e:
            rospy.logerr(f"Error processing frame: {e}")
            self.update_stats(None, 0, success=False)
            
        finally:
            self.processing = False
    
    def analyze_image_with_vlm(self, image):
        """Send image to VLM for navigation analysis"""
        try:
            start_time = time.time()
            
            # Encode image to base64
            _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Prepare VLM request with navigation prompt
            payload = {
                "model": "Qwen2.5-VL-7B",
                "messages": [{
                    "role": "user",
                    "content": [
                        {"type": "text", "text": self.get_navigation_prompt()},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}}
                    ]
                }],
                "max_tokens": 80,
                "temperature": 0.2,  # Lower temperature for more consistent decisions
                "stream": False
            }
            
            # Send to llama.cpp server
            response = requests.post(
                f"{self.llama_server_url}/v1/chat/completions",
                json=payload,
                timeout=15
            )
            
            response_time = time.time() - start_time
            
            if response.status_code == 200:
                result = response.json()
                content = result['choices'][0]['message']['content'].strip()
                decision = self.parse_decision(content)
                
                return {
                    'success': True,
                    'decision': decision,
                    'reasoning': content,
                    'response_time': response_time
                }
            else:
                rospy.logerr(f"VLM server error: {response.status_code}")
                return {'success': False, 'error': f"HTTP {response.status_code}"}
                
        except requests.exceptions.Timeout:
            rospy.logwarn("VLM request timed out")
            return {'success': False, 'error': 'timeout'}
        except Exception as e:
            rospy.logerr(f"VLM analysis error: {e}")
            return {'success': False, 'error': str(e)}
    
    def get_navigation_prompt(self):
        """Get VLM prompt optimized for Duckiebot navigation"""
        return """You are controlling a DuckieBot robot navigating in Duckietown. Analyze this camera view and decide the best navigation action.

ENVIRONMENT:
- Small-scale Duckietown with roads, lanes, and infrastructure
- Yellow dashed lines mark the center of the road
- White solid lines mark road edges  
- Red lines indicate stop lines
- Black road surface with white lane markings

AVAILABLE COMMANDS:
- FORWARD: Continue straight ahead (default for clear roads)
- LEFT: Turn left or change to left lane
- RIGHT: Turn right or change to right lane  
- STOP: Stop immediately for obstacles/safety

NAVIGATION RULES:
1. LANE FOLLOWING: Stay centered between lane markings, follow road curves
2. OBSTACLE AVOIDANCE: Stop for obstacles, other robots, or people
3. TRAFFIC RULES: Stop at red stop lines for 3 seconds
4. DUCKIEBOT DETECTION: Stop for other robots (blue chassis, yellow wheels, rubber duck on top)
5. INTERSECTION HANDLING: Stop, assess, then proceed based on signs/markings
6. SAFETY FIRST: When uncertain, choose STOP

DUCKIEBOT IDENTIFICATION:
- Blue/dark chassis (main robot body)
- Bright yellow wheels on sides
- Yellow rubber duck mounted on top
- LED strips (often colored/blinking)
- Approximately 10-15cm size
- STOP immediately if detected close ahead

LANE CHANGING STRATEGY:
- If obstacle in current lane but adjacent lane clear → change lanes
- Prefer LEFT lane changes (typically less traffic)
- After bypassing obstacle, return to original lane when safe
- Execute smooth lane changes rather than abrupt stops when possible

DECISION PRIORITY:
1. Safety (STOP for immediate danger)
2. Traffic compliance (STOP for red lines)
3. Obstacle avoidance (lane change if safe, otherwise STOP)
4. Forward progress (FORWARD for clear roads)
5. Navigation (LEFT/RIGHT for turns and lane positioning)

Respond with: COMMAND - Brief reasoning (max 40 words)

Examples:
- "FORWARD - Lane clear, centered between markings, no obstacles"
- "LEFT - Obstacle ahead, left lane clear, executing lane change"
- "STOP - Red stop line detected, stopping for traffic compliance"
- "STOP - DuckieBot detected ahead with blue chassis and yellow duck"
- "RIGHT - Following road curve, staying in lane boundaries"
"""
    
    def parse_decision(self, content):
        """Parse VLM response to extract navigation decision"""
        content_upper = content.upper()
        
        # Look for decision keywords in order of priority
        for decision in ['STOP', 'FORWARD', 'LEFT', 'RIGHT']:
            if decision in content_upper:
                return decision
        
        # Default to STOP for safety if no clear decision
        return 'STOP'
    
    def execute_navigation_command(self, decision):
        """Execute navigation command by sending ROS control messages"""
        if self.emergency_stop:
            self.stop_robot()
            return
        
        # Map decisions to velocities
        linear_vel = 0.0
        angular_vel = 0.0
        
        if decision == 'FORWARD':
            linear_vel = self.max_speed
            angular_vel = 0.0
        elif decision == 'LEFT':
            linear_vel = self.max_speed * 0.7  # Reduced speed when turning
            angular_vel = self.max_angular * 0.8  # Left turn
        elif decision == 'RIGHT':
            linear_vel = self.max_speed * 0.7  # Reduced speed when turning
            angular_vel = -self.max_angular * 0.8  # Right turn
        elif decision == 'STOP':
            linear_vel = 0.0
            angular_vel = 0.0
        
        # Publish control commands (using duckie_v2 approach)
        self.publish_control_commands(linear_vel, angular_vel)
    
    def publish_control_commands(self, linear_vel, angular_vel):
        """Publish control commands to all robot interfaces"""
        current_time = rospy.Time.now()
        
        try:
            # 1. Standard ROS Twist message
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            self.cmd_vel_pub.publish(twist_msg)
            
            # 2. DuckieBot Twist2DStamped message
            duckiebot_msg = Twist2DStamped()
            duckiebot_msg.header = Header()
            duckiebot_msg.header.stamp = current_time
            duckiebot_msg.header.frame_id = "base_link"
            duckiebot_msg.v = linear_vel
            duckiebot_msg.omega = angular_vel
            self.duckiebot_vel_pub.publish(duckiebot_msg)
            
            # 3. DuckieBot WheelsCmdStamped message
            # Convert linear and angular velocity to wheel velocities
            vel_left = linear_vel - angular_vel * self.wheel_distance / 2.0
            vel_right = linear_vel + angular_vel * self.wheel_distance / 2.0
            
            wheels_msg = WheelsCmdStamped()
            wheels_msg.header = Header()
            wheels_msg.header.stamp = current_time
            wheels_msg.header.frame_id = "base_link"
            wheels_msg.vel_left = vel_left
            wheels_msg.vel_right = vel_right
            self.wheels_pub.publish(wheels_msg)
            
        except Exception as e:
            rospy.logerr(f"Error publishing control commands: {e}")
    
    def stop_robot(self):
        """Emergency stop - send zero velocities"""
        self.publish_control_commands(0.0, 0.0)
    
    def update_camera_fps(self):
        """Update camera FPS tracking"""
        current_time = time.time()
        if self.last_frame_time > 0:
            fps = 1.0 / (current_time - self.last_frame_time)
            self.fps_tracker.append(fps)
            
            # Keep only last 30 measurements
            if len(self.fps_tracker) > 30:
                self.fps_tracker = self.fps_tracker[-30:]
            
            # Update average every 30 frames
            if len(self.fps_tracker) == 30:
                avg_fps = sum(self.fps_tracker) / len(self.fps_tracker)
                self.stats['camera_fps'] = avg_fps
                rospy.loginfo_throttle(10, f"📹 Camera FPS: {avg_fps:.2f}")
                self.fps_tracker = []
        
        self.last_frame_time = current_time
    
    def update_stats(self, decision, response_time, success):
        """Update performance statistics"""
        if success:
            self.stats['processed_frames'] += 1
            self.stats['successful_decisions'] += 1
            
            # Update average response time
            total = self.stats['successful_decisions']
            self.stats['avg_response_time'] = (
                (self.stats['avg_response_time'] * (total - 1) + response_time) / total
            )
            
            # Update decision counts
            if decision in self.stats['decisions']:
                self.stats['decisions'][decision] += 1
        else:
            self.stats['failed_decisions'] += 1
        
        # Publish performance metrics
        if self.stats['processed_frames'] > 0:
            success_rate = self.stats['successful_decisions'] / self.stats['processed_frames']
            self.performance_pub.publish(Float32(success_rate * 100))
    
    def print_status(self):
        """Print current status and statistics"""
        rospy.loginfo("="*60)
        rospy.loginfo("🤖 DUCKIEBOT VLM NAVIGATOR STATUS")
        rospy.loginfo("="*60)
        rospy.loginfo(f"Robot: {self.robot_name}")
        rospy.loginfo(f"Auto Control: {'ON' if self.auto_control else 'OFF'}")
        rospy.loginfo(f"Emergency Stop: {'ACTIVE' if self.emergency_stop else 'INACTIVE'}")
        rospy.loginfo(f"Current Decision: {self.current_decision}")
        rospy.loginfo(f"Camera FPS: {self.stats['camera_fps']:.2f}")
        rospy.loginfo(f"Processing FPS: {self.processing_fps:.2f}")
        rospy.loginfo(f"Total Frames: {self.stats['total_frames']}")
        rospy.loginfo(f"Processed: {self.stats['processed_frames']}")
        rospy.loginfo(f"Success Rate: {self.stats['successful_decisions']}/{self.stats['processed_frames']}")
        rospy.loginfo(f"Avg Response Time: {self.stats['avg_response_time']:.2f}s")
        
        rospy.loginfo("\n📊 Decision Distribution:")
        for decision, count in self.stats['decisions'].items():
            percentage = 100 * count / max(1, self.stats['successful_decisions'])
            rospy.loginfo(f"  {decision}: {count} ({percentage:.1f}%)")
        rospy.loginfo("="*60)
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("🚀 VLM Navigator running...")
        rospy.loginfo("📋 Commands:")
        rospy.loginfo("  rostopic pub /vlm_navigator/emergency_stop std_msgs/Bool 'data: true'  # Emergency stop")
        rospy.loginfo("  rostopic pub /vlm_navigator/emergency_stop std_msgs/Bool 'data: false' # Resume")
        
        # Print status every 30 seconds
        status_timer = rospy.Timer(rospy.Duration(30), lambda event: self.print_status())
        
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
        finally:
            rospy.loginfo("🛑 VLM Navigator stopped")
            self.stop_robot()

def main():
    parser = argparse.ArgumentParser(description='Duckiebot VLM Navigation System')
    parser.add_argument('robot_name', help='Name of the DuckieBot (e.g., ducky)')
    parser.add_argument('--llama-url', default='http://localhost:8080', 
                       help='URL of the llama.cpp server (default: http://localhost:8080)')
    parser.add_argument('--fps', type=float, default=2.0,
                       help='Processing FPS for VLM analysis (default: 2.0)')
    
    args = parser.parse_args()
    
    # Check if llama.cpp server is running
    try:
        response = requests.get(f"{args.llama_url}/health", timeout=5)
        if response.status_code != 200:
            raise Exception("Server not healthy")
    except:
        print(f"❌ llama.cpp server not running at {args.llama_url}")
        print("Start it with:")
        print("docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \\")
        print("  -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF --host 0.0.0.0 --port 8080 --n-gpu-layers 99")
        return
    
    print("✅ llama.cpp server is running")
    
    # Create and run navigator
    navigator = DuckiebotVLMNavigator(
        robot_name=args.robot_name,
        llama_server_url=args.llama_url,
        processing_fps=args.fps
    )
    
    navigator.run()

if __name__ == '__main__':
    main()
