#!/usr/bin/env python3
"""
Duckiebot VLM Video Streamer
Captures video from Duckiebot camera and streams to VLM Keyboard Control Bridge

This runs on the laptop and:
1. Connects to Duckiebot's camera feed via ROS
2. Streams frames to the VLM keyboard control bridge
3. Monitors performance and provides feedback
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
from cv_bridge import CvBridge, CvBridgeError
import websocket
import argparse
from datetime import datetime

class DuckiebotVLMStreamer:
    def __init__(self, robot_name, bridge_url="http://localhost:8000", use_websocket=True):
        self.robot_name = robot_name
        self.bridge_url = bridge_url
        self.use_websocket = use_websocket
        self.bridge = CvBridge()
        
        # Performance tracking
        self.frame_count = 0
        self.last_frame_time = 0
        self.fps_tracker = []
        
        # WebSocket connection
        self.ws = None
        if use_websocket:
            self.connect_websocket()
        
        # ROS setup
        rospy.init_node('duckiebot_vlm_streamer', anonymous=True)
        
        # Subscribe to camera feed
        self.image_topic = f"/{robot_name}/camera_node/image/compressed"
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo(f"VLM Streamer initialized for robot: {robot_name}")
        rospy.loginfo(f"Subscribing to: {self.image_topic}")
        rospy.loginfo(f"Streaming to: {bridge_url}")
        
    def connect_websocket(self):
        """Connect to the bridge WebSocket"""
        try:
            ws_url = self.bridge_url.replace("http://", "ws://") + "/ws"
            self.ws = websocket.WebSocketApp(
                ws_url,
                on_open=self.on_ws_open,
                on_message=self.on_ws_message,
                on_error=self.on_ws_error,
                on_close=self.on_ws_close
            )
            
            # Run WebSocket in background thread
            ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
            ws_thread.start()
            
        except Exception as e:
            rospy.logerr(f"WebSocket connection failed: {e}")
            self.ws = None
    
    def on_ws_open(self, ws):
        rospy.loginfo("WebSocket connection established")
    
    def on_ws_message(self, ws, message):
        """Handle messages from the bridge"""
        try:
            data = json.loads(message)
            if data.get('status') == 'success':
                decision = data.get('decision', 'UNKNOWN')
                processing_time = data.get('processing_time', 0)
                rospy.loginfo(f"VLM Decision: {decision} ({processing_time:.2f}s)")
        except Exception as e:
            rospy.logerr(f"Error parsing WebSocket message: {e}")
    
    def on_ws_error(self, ws, error):
        rospy.logerr(f"WebSocket error: {error}")
    
    def on_ws_close(self, ws, close_status_code, close_msg):
        rospy.logwarn("WebSocket connection closed, attempting to reconnect...")
        time.sleep(3)
        self.connect_websocket()
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                rospy.logerr("Failed to decode image")
                return
            
            # Track FPS
            self.update_fps()
            
            # Encode image to base64
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Prepare payload
            payload = {
                "image_base64": image_base64,
                "timestamp": time.time(),
                "robot_name": self.robot_name
            }
            
            # Send to bridge
            if self.use_websocket and self.ws and self.ws.sock and self.ws.sock.connected:
                self.ws.send(json.dumps(payload))
            else:
                # Fallback to HTTP POST
                self.send_via_http(payload)
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def send_via_http(self, payload):
        """Send frame via HTTP POST"""
        try:
            response = requests.post(
                f"{self.bridge_url}/process_frame",
                json=payload,
                timeout=5
            )
            
            if response.status_code == 200:
                data = response.json()
                if data.get('status') == 'success':
                    decision = data.get('decision', 'UNKNOWN')
                    rospy.loginfo(f"VLM Decision: {decision}")
            else:
                rospy.logerr(f"Bridge returned status {response.status_code}")
                
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"HTTP request failed: {e}")
    
    def update_fps(self):
        """Update FPS tracking"""
        current_time = time.time()
        if self.last_frame_time > 0:
            fps = 1.0 / (current_time - self.last_frame_time)
            self.fps_tracker.append(fps)
            
            # Keep only last 30 FPS measurements
            if len(self.fps_tracker) > 30:
                self.fps_tracker = self.fps_tracker[-30:]
            
            # Log average FPS every 30 frames
            if len(self.fps_tracker) == 30:
                avg_fps = sum(self.fps_tracker) / len(self.fps_tracker)
                rospy.loginfo(f"Camera FPS: {avg_fps:.2f}")
                self.fps_tracker = []
        
        self.last_frame_time = current_time
        self.frame_count += 1
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("VLM Streamer running...")
        rospy.spin()

def main():
    parser = argparse.ArgumentParser(description='Duckiebot VLM Video Streamer')
    parser.add_argument('robot_name', help='Name of the Duckiebot')
    parser.add_argument('--bridge-url', default='http://localhost:8000', 
                       help='URL of the VLM keyboard control bridge')
    parser.add_argument('--no-websocket', action='store_true',
                       help='Use HTTP POST instead of WebSocket')
    
    args = parser.parse_args()
    
    streamer = DuckiebotVLMStreamer(
        robot_name=args.robot_name,
        bridge_url=args.bridge_url,
        use_websocket=not args.no_websocket
    )
    
    try:
        streamer.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
