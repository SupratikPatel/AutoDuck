#!/usr/bin/env python3
"""
Simple Screen Capture VLM Control for Duckiebot
Uses desktop screen capture instead of complex video streaming

Workflow:
1. Launch Duckietown camera GUI: dts start_gui_tools ROBOT_NAME -> rqt_image_view
2. Launch keyboard control: dts duckiebot keyboard_control ROBOT_NAME  
3. Run this script to capture the camera GUI area and control via VLM
4. Select the camera window area for capture
5. VLM analyzes screen capture and sends keyboard commands

Much simpler than FastAPI/ROS integration!
"""

import cv2
import numpy as np
import requests
import base64
import time
import threading
import json
import tkinter as tk
from tkinter import messagebox
from datetime import datetime
import pyautogui
import pygetwindow as gw
import keyboard
import mss
import argparse

class ScreenCaptureVLM:
    def __init__(self, llama_server_url="http://localhost:8080", capture_fps=2):
        self.llama_server_url = llama_server_url
        self.capture_fps = capture_fps
        self.capture_interval = 1.0 / capture_fps
        
        # Screen capture settings
        self.monitor = None
        self.capture_region = None
        self.sct = mss.mss()
        
        # VLM and control settings
        self.auto_control = True  # Enable auto control by default for testing
        self.last_decision = None
        self.last_capture_time = 0
        
        # Performance tracking
        self.stats = {
            'total_captures': 0,
            'successful_decisions': 0,
            'failed_decisions': 0,
            'avg_response_time': 0.0,
            'decisions': {'FORWARD': 0, 'LEFT': 0, 'RIGHT': 0, 'STOP': 0, 'BACKWARD': 0}
        }
        
        # Key mapping for Duckietown keyboard control
        self.decision_to_key = {
            'FORWARD': 'up',     # Arrow keys for Duckietown GUI
            'LEFT': 'left', 
            'RIGHT': 'right',
            'STOP': 'down',      # Down arrow to stop
            'BACKWARD': 'down'   # Same as stop - move backward/stop
        }
        
        # Alternative WASD mapping (if arrow keys don't work)
        self.decision_to_wasd = {
            'FORWARD': 'w',
            'LEFT': 'a', 
            'RIGHT': 'd',
            'STOP': 's',
            'BACKWARD': 's'  # Same as stop in WASD mode
        }
        
        self.use_wasd = False  # Use arrow keys for Duckietown compatibility
        
        print("🎥 Simple Screen Capture VLM Controller")
        print("📋 Setup Instructions:")
        print("1. Launch camera view: dts start_gui_tools ROBOT_NAME")
        print("2. Run: rqt_image_view (select camera_node/image/compressed)")
        print("3. Launch control: dts duckiebot keyboard_control ROBOT_NAME")
        print("4. Run this script and select the camera window area")
        
    def setup_capture_region(self):
        """Interactive setup of screen capture region"""
        print("\n🖼️ Setting up screen capture region...")
        
        # Method 1: Try to find rqt_image_view window automatically
        windows = gw.getWindowsWithTitle('rqt_image_view')
        if windows:
            window = windows[0]
            print(f"✅ Found rqt_image_view window: {window.title}")
            
            # Get window position and size
            left, top, width, height = window.left, window.top, window.width, window.height
            self.capture_region = {'top': top, 'left': left, 'width': width, 'height': height}
            
            print(f"📐 Capture region: {left}, {top}, {width}x{height}")
            return True
        
        # Method 2: Try to find keyboard control window
        kb_windows = gw.getWindowsWithTitle('keyboard')
        if kb_windows:
            print(f"✅ Found keyboard control window: {kb_windows[0].title}")
        
        # Method 3: Manual selection
        print("❌ Could not auto-detect rqt_image_view window")
        print("📋 Manual setup:")
        print("1. Make sure rqt_image_view window is visible")
        print("2. Press ENTER to start manual selection")
        input("Press ENTER when ready...")
        
        return self.manual_region_selection()
    
    def manual_region_selection(self):
        """Manual region selection using tkinter"""
        try:
            root = tk.Tk()
            root.withdraw()  # Hide main window
            
            messagebox.showinfo(
                "Screen Capture Setup", 
                "Instructions:\n"
                "1. Click OK to start selection\n"
                "2. Move cursor to TOP-LEFT of camera view\n"
                "3. Press SPACE to mark first corner\n"
                "4. Move cursor to BOTTOM-RIGHT of camera view\n"
                "5. Press SPACE to mark second corner\n"
                "6. Press ESC to cancel"
            )
            
            root.destroy()
            
            # Get two corner points
            print("🎯 Move cursor to TOP-LEFT corner of camera view and press SPACE...")
            point1 = self.wait_for_space_key()
            if not point1:
                return False
                
            print("🎯 Move cursor to BOTTOM-RIGHT corner of camera view and press SPACE...")
            point2 = self.wait_for_space_key()
            if not point2:
                return False
            
            # Calculate region
            left = min(point1[0], point2[0])
            top = min(point1[1], point2[1])
            width = abs(point2[0] - point1[0])
            height = abs(point2[1] - point1[1])
            
            self.capture_region = {'top': top, 'left': left, 'width': width, 'height': height}
            
            print(f"✅ Capture region set: {left}, {top}, {width}x{height}")
            return True
            
        except Exception as e:
            print(f"❌ Region selection failed: {e}")
            return False
    
    def wait_for_space_key(self):
        """Wait for user to press space and return cursor position"""
        while True:
            try:
                if keyboard.is_pressed('space'):
                    pos = pyautogui.position()
                    print(f"✅ Point selected: {pos}")
                    time.sleep(0.5)  # Debounce
                    return pos
                elif keyboard.is_pressed('esc'):
                    print("❌ Selection cancelled")
                    return None
                time.sleep(0.1)
            except KeyboardInterrupt:
                return None
    
    def capture_screen_region(self):
        """Capture the selected screen region"""
        if not self.capture_region:
            return None
            
        try:
            # Capture screen region
            screenshot = self.sct.grab(self.capture_region)
            
            # Convert to OpenCV format
            img = np.array(screenshot)
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            
            return img
            
        except Exception as e:
            print(f"❌ Screen capture failed: {e}")
            return None
    
    def check_llama_server(self):
        """Check if llama.cpp server is running"""
        try:
            response = requests.get(f"{self.llama_server_url}/health", timeout=2)
            return response.status_code == 200
        except:
            return False
    
    def get_driving_prompt(self):
        """Get VLM prompt for driving decisions"""
        return """You are controlling a small, slow-moving Duckiebot robot car in a miniature Duckietown environment. Analyze this camera view and decide the best driving action.

CRITICAL SCALE AWARENESS: 
- This is a MINIATURE robot moving very slowly in a small-scale environment
- Objects that appear large or "dangerous" are actually still FAR AWAY due to the small scale
- You can continue FORWARD even with obstacles visible ahead - they are much farther than they appear
- Only STOP when obstacles are VERY CLOSE (filling most of the camera view)

Available commands:
- FORWARD: Continue straight ahead (preferred action for clear or distant obstacles)
- LEFT: Turn left (when path ahead is blocked and left side is clear)
- RIGHT: Turn right (when path ahead is blocked and right side is clear)  
- STOP: Stop for extremely close obstacles, immediate danger, or Duckietown inhabitants (ducks/robots within 5cm)
- BACKWARD: Move backward to create space, then reassess (use when stuck or in deadlock)

Duckietown Environment Rules:
1. Stay between yellow dashed line (left lane boundary) and white solid line (right lane boundary)
2. Red lines indicate stop lines - stop before them
3. Follow the black road surface with white lane markings
4. PROTECT DUCKIETOWN INHABITANTS: Duck toys, other robots, pedestrians, small animals
5. Traffic signs and infrastructure are proportionally large for the small robot

🦆 DUCK PROTECTION PROTOCOL:
- DUCK TOYS: Yellow rubber ducks or any duck-like objects - STOP immediately if within 5cm
- DUCKIEBOTS: Small robotic vehicles with distinctive features:
  * Blue/dark blue/black main chassis (rectangular robot body)
  * Bright yellow wheels on sides
  * Yellow rubber duck mounted on top (signature duckiebot feature)
  * LED strip lights (often colored/blinking) 
  * Visible electronic components, cameras, or sensors
  * Approximately 10-15cm in size
  * STOP if within 5cm, wait for them to pass or consider lane change if safe
- OTHER ROBOTS: Any moving robot, autonomous vehicle, or robotic platform - STOP if within 5cm, wait for them to pass
- TRAFFIC SIGNS: Stop signs, yield signs - respect and stop appropriately
- PEDESTRIANS: Any people or figures - STOP immediately if within 5cm
- RED LINES: Red stop lines or red tape - STOP when within 5cm and wait for 5 seconds before proceeding
- When in doubt about small moving objects, prioritize safety and STOP

Navigation Strategy:
- DEFAULT to FORWARD unless path is completely blocked
- Objects in distance = FORWARD (they're farther than they look)
- 🦆 DUCK SAFETY OVERRIDE: Always STOP for duck toys, other robots, or people within 5cm

LANE CHANGING PROTOCOL (prioritized for obstacle avoidance):
- When encountering duckiebots or obstacles in current lane:
  1. First assess if LEFT lane is clear and safe to change into
  2. If left lane blocked, check if RIGHT lane is clear and safe
  3. Execute lane change to avoid obstacle rather than just stopping
  4. Since robot is mostly in right lane, LEFT lane is typically clearer
  5. After passing obstacle, return to original lane when safe

- Only turn LEFT/RIGHT when:
  * Path directly ahead is blocked AND turn direction is clear, OR
  * Executing strategic lane change to bypass obstacle
- STOP for immediate very close obstacles, red stop lines (wait 5 seconds), or Duckietown inhabitants
- Use BACKWARD when completely stuck or need to create space for turning
- Balance forward progress with Duckietown safety - protect the ducks!

Respond with: COMMAND - Brief reasoning

Examples:
- "FORWARD - Road clear, obstacle distant, no ducks or robots nearby"
- "STOP - Duck toy detected within 5cm, protecting Duckietown inhabitant"
- "STOP - Duckiebot detected: blue chassis with yellow wheels and rubber duck on top, too close for safe passing"
- "STOP - Red stop line within 5cm, stopping for 5 seconds as required"
- "LEFT - Duckiebot ahead in current lane, left lane clear, executing lane change to bypass obstacle"
- "RIGHT - Obstacle in current lane, left blocked but right lane clear, changing lanes to avoid"
- "LEFT - Path blocked ahead, left side clear, no ducks in turn path" """
    
    def analyze_image_with_vlm(self, image):
        """Send image to VLM for analysis"""
        if image is None:
            return None
            
        try:
            start_time = time.time()
            
            # Encode image to base64
            _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Prepare VLM request
            payload = {
                "model": "Qwen2.5-VL-7B",
                "messages": [{
                    "role": "user", 
                    "content": [
                        {"type": "text", "text": self.get_driving_prompt()},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}}
                    ]
                }],
                "max_tokens": 50,
                "temperature": 0.3,
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
                
                # Update stats
                self.stats['successful_decisions'] += 1
                self.stats['avg_response_time'] = (
                    (self.stats['avg_response_time'] * (self.stats['successful_decisions'] - 1) + response_time) /
                    self.stats['successful_decisions']
                )
                if decision in self.stats['decisions']:
                    self.stats['decisions'][decision] += 1
                
                return {
                    'decision': decision,
                    'reasoning': content,
                    'response_time': response_time,
                    'timestamp': datetime.now().isoformat()
                }
            else:
                self.stats['failed_decisions'] += 1
                print(f"❌ VLM server error: {response.status_code}")
                return None
                
        except Exception as e:
            self.stats['failed_decisions'] += 1
            print(f"❌ VLM analysis error: {e}")
            return None
    
    def parse_decision(self, content):
        """Parse VLM response to extract decision"""
        content_upper = content.upper()
        
        # Look for decision keywords
        for decision in ['FORWARD', 'LEFT', 'RIGHT', 'STOP', 'BACKWARD']:
            if decision in content_upper:
                return decision
        
        # Default to STOP for safety
        return 'STOP'
    
    def send_keyboard_command(self, decision):
        """Send keyboard command based on VLM decision"""
        try:
            # Choose key mapping
            key_map = self.decision_to_wasd if self.use_wasd else self.decision_to_key
            
            if decision in key_map:
                key = key_map[decision]
                
                # Send key press
                pyautogui.keyDown(key)
                time.sleep(0.1)  # Brief press
                pyautogui.keyUp(key)
                
                print(f"🎮 Sent command: {decision} ({key})")
                return True
            else:
                print(f"❌ Unknown decision: {decision}")
                return False
                
        except Exception as e:
            print(f"❌ Keyboard command failed: {e}")
            return False
    
    def run_capture_loop(self):
        """Main capture and control loop"""
        print("\n🚀 Starting VLM control loop")
        print("📋 Controls:")
        print("  SPACE: Toggle auto control")
        print("  T: Toggle WASD/Arrow keys")
        print("  Q: Quit")
        print("  M: Manual single capture")
        
        try:
            while True:
                current_time = time.time()
                
                # Check keyboard input
                if keyboard.is_pressed('q'):
                    break
                elif keyboard.is_pressed('space'):
                    self.auto_control = not self.auto_control
                    print(f"🎮 Auto control: {'ON' if self.auto_control else 'OFF'}")
                    time.sleep(0.5)  # Debounce
                elif keyboard.is_pressed('t'):
                    self.use_wasd = not self.use_wasd
                    key_type = "WASD" if self.use_wasd else "Arrow keys"
                    print(f"⌨️ Key mapping: {key_type}")
                    time.sleep(0.5)  # Debounce
                elif keyboard.is_pressed('m'):
                    # Manual single capture
                    self.process_single_frame()
                    time.sleep(0.5)  # Debounce
                
                # Auto capture based on FPS
                if (self.auto_control and 
                    current_time - self.last_capture_time >= self.capture_interval):
                    self.process_single_frame()
                    self.last_capture_time = current_time
                
                time.sleep(0.1)  # Small delay to prevent high CPU usage
                
        except KeyboardInterrupt:
            pass
        finally:
            self.print_final_stats()
    
    def process_single_frame(self):
        """Process a single frame: capture, analyze, command"""
        # Capture screen
        image = self.capture_screen_region()
        if image is None:
            return
        
        self.stats['total_captures'] += 1
        
        # Analyze with VLM
        result = self.analyze_image_with_vlm(image)
        if result:
            decision = result['decision']
            reasoning = result['reasoning']
            response_time = result['response_time']
            
            print(f"🤖 {decision} | {response_time:.2f}s | {reasoning[:50]}...")
            
            # Send keyboard command if auto control is on
            if self.auto_control:
                self.send_keyboard_command(decision)
            
            self.last_decision = result
    
    def print_final_stats(self):
        """Print final statistics"""
        print("\n" + "="*50)
        print("📊 FINAL SESSION STATISTICS")
        print("="*50)
        print(f"Total captures: {self.stats['total_captures']}")
        print(f"Successful decisions: {self.stats['successful_decisions']}")
        print(f"Failed decisions: {self.stats['failed_decisions']}")
        print(f"Average response time: {self.stats['avg_response_time']:.2f}s")
        
        print("\n🎮 Decision Distribution:")
        for decision, count in self.stats['decisions'].items():
            percentage = 100 * count / max(1, self.stats['successful_decisions'])
            print(f"  {decision}: {count} ({percentage:.1f}%)")
        
        print("="*50)
    
    def run(self):
        """Main run method"""
        # Check llama.cpp server
        if not self.check_llama_server():
            print("❌ llama.cpp server not running!")
            print("Start it with: docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \\")
            print("    -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF --host 0.0.0.0 --port 8080 --n-gpu-layers 99")
            return
        
        print("✅ llama.cpp server is running")
        
        # Setup screen capture region
        if not self.setup_capture_region():
            print("❌ Failed to setup capture region")
            return
        
        # Test capture
        test_image = self.capture_screen_region()
        if test_image is not None:
            print(f"✅ Screen capture working: {test_image.shape}")
            # Optionally show test image
            cv2.imwrite('test_capture.jpg', test_image)
            print("💾 Test capture saved as test_capture.jpg")
        else:
            print("❌ Screen capture failed")
            return
        
        # Start main loop
        self.run_capture_loop()

def main():
    parser = argparse.ArgumentParser(description='Simple Screen Capture VLM Control')
    parser.add_argument('--fps', type=float, default=2.0, help='Capture FPS (default: 2.0)')
    parser.add_argument('--llama-url', default='http://localhost:8080', help='llama.cpp server URL')
    
    args = parser.parse_args()
    
    controller = ScreenCaptureVLM(
        llama_server_url=args.llama_url,
        capture_fps=args.fps
    )
    
    controller.run()

if __name__ == "__main__":
    main()
