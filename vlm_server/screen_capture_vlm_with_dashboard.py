#!/usr/bin/env python3
"""
Enhanced VLM with Web Dashboard
Combines screen capture functionality with real-time web monitoring
Similar to llamacpp_autoduck.py but for screen capture approach

Dashboard: http://localhost:3000
VLM Server: http://localhost:8080
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
from flask import Flask, render_template_string, jsonify, Response
import io

class ScreenCaptureVLMWithDashboard:
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
        self.continuous_capture = False
        self.last_decision = None
        self.last_capture_time = 0
        self.last_captured_image = None
        
        # Performance tracking
        self.stats = {
            'total_captures': 0,
            'successful_decisions': 0,
            'failed_decisions': 0,
            'avg_response_time': 0.0,
            'current_fps': 0.0,
            'decisions': {'FORWARD': 0, 'LEFT': 0, 'RIGHT': 0, 'STOP': 0, 'BACKWARD': 0},
            'decision_history': []
        }
        
        # Key mapping for Duckietown keyboard control
        self.decision_to_key = {
            'FORWARD': 'up',     # Arrow keys for Duckietown GUI
            'LEFT': 'left', 
            'RIGHT': 'right',
            'STOP': 'down',      # Down arrow to stop
            'BACKWARD': 'down'   # Same as stop - move backward/stop
        }
        
        # Alternative WASD mapping
        self.decision_to_wasd = {
            'FORWARD': 'w',
            'LEFT': 'a', 
            'RIGHT': 'd',
            'STOP': 's',
            'BACKWARD': 's'  # Same as stop in WASD mode
        }
        
        self.use_wasd = False  # Use arrow keys for Duckietown compatibility
        
        # Flask dashboard
        self.app = Flask(__name__)
        self.setup_dashboard_routes()
        
        print("🎥 VLM with Dashboard")
        print("📊 Dashboard will be available at: http://localhost:3000")
        print("🧠 VLM Server should be running at: http://localhost:8080")
        
    def setup_dashboard_routes(self):
        """Setup Flask routes for the dashboard"""
        
        @self.app.route('/')
        def dashboard():
            return render_template_string(DASHBOARD_TEMPLATE)
        
        @self.app.route('/api/stats')
        def get_stats():
            return jsonify(self.stats)
        
        @self.app.route('/api/latest')
        def get_latest():
            if self.last_decision:
                return jsonify({
                    'result': self.last_decision,
                    'timestamp': datetime.now().isoformat(),
                    'auto_control': self.auto_control,
                    'continuous_capture': self.continuous_capture
                })
            return jsonify({'result': None})
        
        @self.app.route('/api/toggle_auto')
        def toggle_auto():
            self.auto_control = not self.auto_control
            return jsonify({'auto_control': self.auto_control})
        
        @self.app.route('/api/toggle_continuous')
        def toggle_continuous():
            self.continuous_capture = not self.continuous_capture
            return jsonify({'continuous_capture': self.continuous_capture})
        
        @self.app.route('/api/capture_single')
        def capture_single():
            self.process_single_frame()
            return jsonify({'status': 'captured'})
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_frames(), 
                          mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def generate_frames(self):
        """Generate video frames for dashboard"""
        while True:
            if self.last_captured_image is not None:
                # Add overlay with current decision
                frame = self.last_captured_image.copy()
                self.add_overlay(frame)
                
                # Encode frame
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.1)
    
    def add_overlay(self, frame):
        """Add performance overlay to frame"""
        if not self.last_decision:
            return
        
        # Overlay background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (350, 140), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Decision text
        decision = self.last_decision.get('decision', 'UNKNOWN')
        color = {
            'FORWARD': (0, 255, 0),    # Green
            'LEFT': (0, 255, 255),     # Yellow  
            'RIGHT': (255, 0, 255),    # Magenta
            'STOP': (0, 0, 255),       # Red
        }.get(decision, (255, 255, 255))
        
        cv2.putText(frame, f"Decision: {decision}", (20, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        # Status indicators
        auto_status = "AUTO ON" if self.auto_control else "AUTO OFF"
        cv2.putText(frame, auto_status, (20, 65), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.auto_control else (0, 0, 255), 2)
        
        continuous_status = "CONTINUOUS" if self.continuous_capture else "MANUAL"
        cv2.putText(frame, continuous_status, (20, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.continuous_capture else (255, 255, 255), 1)
        
        # Performance stats
        rt = self.last_decision.get('response_time', 0)
        fps = 1.0 / rt if rt > 0 else 0
        
        cv2.putText(frame, f"Response: {rt:.2f}s", (20, 115), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Total: {self.stats['total_captures']}", (20, 135), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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
        
        # Method 2: Manual selection
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
            
            # Store for dashboard
            self.last_captured_image = img.copy()
            
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
- STOP: Stop for extremely close obstacles, immediate danger, or Duckietown inhabitants (ducks/robots within 10cm)
- BACKWARD: Move backward to create space, then reassess (use when stuck or in deadlock)

Duckietown Environment Rules:
1. Stay between yellow dashed line (left lane boundary) and white solid line (right lane boundary)
2. Red lines indicate stop lines - stop before them
3. Follow the black road surface with white lane markings
4. PROTECT DUCKIETOWN INHABITANTS: Duck toys, other robots, pedestrians, small animals
5. Traffic signs and infrastructure are proportionally large for the small robot

🦆 CRITICAL STOP CONDITIONS (within 10cm - MANDATORY):
1. DUCKIEBOTS: Small robotic vehicles with these EXACT distinctive features:
   * Blue/dark blue/black main chassis (rectangular robot body)
   * Bright yellow wheels on sides
   * Yellow rubber duck mounted on top (signature duckiebot feature)
   * LED strip lights (often colored/blinking) 
   * Visible electronic components, cameras, or sensors
   * Approximately 10-15cm in size
   * STOP immediately if within 10cm, wait for them to pass or consider lane change if safe

2. YELLOW RUBBER DUCKS: Small yellow duck toys or any duck-like objects
   * Classic yellow rubber duck shape and color
   * Any small yellow duck toys on the road or nearby
   * Duck-shaped objects regardless of exact size
   * STOP immediately if within 10cm to protect Duckietown inhabitants

3. RED LINES: Red stop lines or red tape markings
   * Red colored lines across the road surface
   * Red tape or paint markings indicating stop zones
   * Any red horizontal markings on the road
   * STOP when within 10cm and wait for 5 seconds before proceeding

4. OTHER SAFETY ITEMS:
   * OTHER ROBOTS: Any moving robot, autonomous vehicle, or robotic platform - STOP if within 10cm
   * TRAFFIC SIGNS: Stop signs, yield signs - respect and stop appropriately
   * PEDESTRIANS: Any people or figures - STOP immediately if within 10cm
   * When in doubt about small moving objects, prioritize safety and STOP

Navigation Strategy:
- DEFAULT to FORWARD unless path is completely blocked
- Objects in distance = FORWARD (they're farther than they look due to miniature scale)
- 🦆 SAFETY OVERRIDE: Always STOP for duckiebots, yellow ducks, red lines, or people within 10cm
- Only STOP when obstacles are VERY CLOSE (within 10cm or filling most of camera view)

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
- "FORWARD - Road clear, obstacles distant, no duckiebots, yellow ducks, or red lines nearby"
- "STOP - Yellow rubber duck detected within 10cm, protecting Duckietown inhabitant"
- "STOP - Duckiebot detected: blue chassis with yellow wheels and rubber duck on top, within 10cm - too close for safe passing"
- "STOP - Red stop line within 10cm, stopping for 5 seconds as required"
- "STOP - Multiple yellow duck toys within 10cm, ensuring Duckietown safety"
- "LEFT - Duckiebot ahead in current lane beyond 10cm, left lane clear, executing lane change to bypass obstacle"
- "RIGHT - Obstacle in current lane, left blocked but right lane clear, changing lanes to avoid"
- "LEFT - Path blocked ahead, left side clear, no ducks or robots in turn path"
- "FORWARD - Distant duckiebot visible but far away (>10cm), safe to continue" """
    
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
                self.stats['current_fps'] = 1.0 / response_time if response_time > 0 else 0
                
                if decision in self.stats['decisions']:
                    self.stats['decisions'][decision] += 1
                
                # Add to decision history
                decision_data = {
                    'decision': decision,
                    'reasoning': content,
                    'response_time': response_time,
                    'timestamp': datetime.now().isoformat()
                }
                
                self.stats['decision_history'].append(decision_data)
                if len(self.stats['decision_history']) > 50:
                    self.stats['decision_history'] = self.stats['decision_history'][-50:]
                
                return decision_data
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
    
    def run_capture_loop(self):
        """Main capture and control loop"""
        print("\n🚀 Starting VLM control loop")
        print("📊 Dashboard available at: http://localhost:3000")
        print("📋 Controls:")
        print("  SPACE: Toggle auto control")
        print("  C: Toggle continuous capture")
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
                elif keyboard.is_pressed('c'):
                    self.continuous_capture = not self.continuous_capture
                    print(f"📹 Continuous capture: {'ON' if self.continuous_capture else 'OFF'}")
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
                
                # Continuous capture based on FPS
                if (self.continuous_capture and 
                    current_time - self.last_capture_time >= self.capture_interval):
                    self.process_single_frame()
                    self.last_capture_time = current_time
                
                time.sleep(0.1)  # Small delay to prevent high CPU usage
                
        except KeyboardInterrupt:
            pass
        finally:
            self.print_final_stats()
    
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
    
    def run_dashboard(self):
        """Run the web dashboard server"""
        print("🌐 Dashboard starting at http://localhost:3000")
        self.app.run(host='0.0.0.0', port=3000, debug=False, threaded=True)
    
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
            # Save test image
            cv2.imwrite('test_capture.jpg', test_image)
            print("💾 Test capture saved as test_capture.jpg")
        else:
            print("❌ Screen capture failed")
            return
        
        # Start dashboard in background
        dashboard_thread = threading.Thread(target=self.run_dashboard, daemon=True)
        dashboard_thread.start()
        
        # Start main capture loop
        self.run_capture_loop()

# Dashboard HTML Template (similar to llamacpp_autoduck.py but for screen capture)
DASHBOARD_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>VLM Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #1e1e1e; color: white; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 30px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 20px; border-radius: 10px; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }
        .card { background: #2d2d2d; border-radius: 10px; padding: 20px; }
        .video-container { position: relative; }
        .video-feed { width: 100%; border-radius: 10px; }
        .controls { display: flex; gap: 10px; margin: 20px 0; justify-content: center; }
        .btn { background: #4CAF50; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; font-size: 16px; }
        .btn:hover { background: #45a049; }
        .btn.stop { background: #f44336; }
        .btn.stop:hover { background: #da190b; }
        .btn.active { background: #2196F3; }
        .stats-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-bottom: 20px; }
        .stat-item { background: #3d3d3d; padding: 15px; border-radius: 8px; text-align: center; }
        .stat-number { font-size: 24px; font-weight: bold; color: #4CAF50; }
        .stat-label { font-size: 14px; color: #ccc; margin-top: 5px; }
        .decision-display { font-size: 48px; font-weight: bold; text-align: center; padding: 20px; border-radius: 10px; margin-bottom: 20px; }
        .decision-FORWARD { background: #4CAF50; color: white; }
        .decision-LEFT { background: #FF9800; color: white; }
        .decision-RIGHT { background: #9C27B0; color: white; }
        .decision-STOP { background: #F44336; color: white; }
        .reasoning { background: #3d3d3d; padding: 15px; border-radius: 8px; font-family: monospace; }
        .status-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
        .status-on { background: #4CAF50; }
        .status-off { background: #f44336; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🎥 VLM Dashboard</h1>
            <p>Real-time Duckiebot Control via Screen Capture + Vision Language Model</p>
        </div>
        
        <div class="controls">
            <button id="toggle-auto" class="btn" onclick="toggleAuto()">Auto Control: OFF</button>
            <button id="toggle-continuous" class="btn" onclick="toggleContinuous()">Continuous Capture: OFF</button>
            <button class="btn" onclick="captureSingle()">📸 Capture Single Frame</button>
        </div>
        
        <div class="grid">
            <div class="card">
                <h3>📹 Screen Capture Feed</h3>
                <div class="video-container">
                    <img class="video-feed" src="/video_feed" alt="Screen Capture Feed">
                </div>
            </div>
            
            <div class="card">
                <h3>🧠 Current Analysis</h3>
                <div id="decision-display" class="decision-display">
                    Waiting for analysis...
                </div>
                <div id="reasoning" class="reasoning">
                    AI reasoning will appear here...
                </div>
                <div id="analysis-time" style="text-align: center; margin-top: 10px; color: #ccc;">
                    Response time: --
                </div>
            </div>
        </div>
        
        <div class="card">
            <h3>📊 Performance Metrics</h3>
            <div class="stats-grid">
                <div class="stat-item">
                    <div id="total-captures" class="stat-number">0</div>
                    <div class="stat-label">Total Captures</div>
                </div>
                <div class="stat-item">
                    <div id="success-rate" class="stat-number">0%</div>
                    <div class="stat-label">Success Rate</div>
                </div>
                <div class="stat-item">
                    <div id="avg-response" class="stat-number">0.00s</div>
                    <div class="stat-label">Avg Response Time</div>
                </div>
                <div class="stat-item">
                    <div id="current-fps" class="stat-number">0.00</div>
                    <div class="stat-label">Current FPS</div>
                </div>
            </div>
            
            <h4>🎮 Decision Distribution</h4>
            <div class="stats-grid">
                <div class="stat-item">
                    <div id="forward-count" class="stat-number">0</div>
                    <div class="stat-label">FORWARD</div>
                </div>
                <div class="stat-item">
                    <div id="left-count" class="stat-number">0</div>
                    <div class="stat-label">LEFT</div>
                </div>
                <div class="stat-item">
                    <div id="right-count" class="stat-number">0</div>
                    <div class="stat-label">RIGHT</div>
                </div>
                <div class="stat-item">
                    <div id="stop-count" class="stat-number">0</div>
                    <div class="stat-label">STOP</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h3>⚙️ System Status</h3>
            <p><span id="auto-status" class="status-indicator status-off"></span>Auto Control: <span id="auto-text">OFF</span></p>
            <p><span id="continuous-status" class="status-indicator status-off"></span>Continuous Capture: <span id="continuous-text">OFF</span></p>
        </div>
    </div>

    <script>
        let autoControl = false;
        let continuousCapture = false;
        
        function updateData() {
            // Update latest analysis
            fetch('/api/latest')
                .then(response => response.json())
                .then(data => {
                    if (data.result) {
                        const decision = data.result.decision;
                        const reasoning = data.result.reasoning;
                        const responseTime = data.result.response_time;
                        
                        const decisionEl = document.getElementById('decision-display');
                        decisionEl.textContent = decision;
                        decisionEl.className = 'decision-display decision-' + decision;
                        
                        document.getElementById('reasoning').textContent = reasoning;
                        document.getElementById('analysis-time').textContent = 
                            `Response time: ${responseTime.toFixed(2)}s`;
                    }
                    
                    // Update status indicators
                    autoControl = data.auto_control;
                    continuousCapture = data.continuous_capture;
                    updateStatusIndicators();
                });
            
            // Update stats
            fetch('/api/stats')
                .then(response => response.json())
                .then(stats => {
                    document.getElementById('total-captures').textContent = stats.total_captures;
                    
                    const successRate = stats.total_captures > 0 ? 
                        (100 * stats.successful_decisions / stats.total_captures).toFixed(1) : 0;
                    document.getElementById('success-rate').textContent = successRate + '%';
                    
                    document.getElementById('avg-response').textContent = stats.avg_response_time.toFixed(2) + 's';
                    document.getElementById('current-fps').textContent = stats.current_fps.toFixed(2);
                    
                    // Decision counts
                    document.getElementById('forward-count').textContent = stats.decisions.FORWARD;
                    document.getElementById('left-count').textContent = stats.decisions.LEFT;
                    document.getElementById('right-count').textContent = stats.decisions.RIGHT;
                    document.getElementById('stop-count').textContent = stats.decisions.STOP;
                });
        }
        
        function updateStatusIndicators() {
            // Auto control status
            const autoBtn = document.getElementById('toggle-auto');
            const autoStatus = document.getElementById('auto-status');
            const autoText = document.getElementById('auto-text');
            
            autoBtn.textContent = `Auto Control: ${autoControl ? 'ON' : 'OFF'}`;
            autoBtn.className = autoControl ? 'btn active' : 'btn';
            autoStatus.className = autoControl ? 'status-indicator status-on' : 'status-indicator status-off';
            autoText.textContent = autoControl ? 'ON' : 'OFF';
            
            // Continuous capture status
            const continuousBtn = document.getElementById('toggle-continuous');
            const continuousStatus = document.getElementById('continuous-status');
            const continuousText = document.getElementById('continuous-text');
            
            continuousBtn.textContent = `Continuous Capture: ${continuousCapture ? 'ON' : 'OFF'}`;
            continuousBtn.className = continuousCapture ? 'btn active' : 'btn';
            continuousStatus.className = continuousCapture ? 'status-indicator status-on' : 'status-indicator status-off';
            continuousText.textContent = continuousCapture ? 'ON' : 'OFF';
        }
        
        function toggleAuto() {
            fetch('/api/toggle_auto')
                .then(response => response.json())
                .then(data => {
                    autoControl = data.auto_control;
                    updateStatusIndicators();
                });
        }
        
        function toggleContinuous() {
            fetch('/api/toggle_continuous')
                .then(response => response.json())
                .then(data => {
                    continuousCapture = data.continuous_capture;
                    updateStatusIndicators();
                });
        }
        
        function captureSingle() {
            fetch('/api/capture_single')
                .then(response => response.json())
                .then(data => {
                    console.log('Single capture triggered');
                });
        }
        
        // Update every second
        setInterval(updateData, 1000);
        updateData(); // Initial load
    </script>
</body>
</html>
"""

def main():
    parser = argparse.ArgumentParser(description='VLM with Dashboard')
    parser.add_argument('--fps', type=float, default=2.0, help='Capture FPS (default: 2.0)')
    parser.add_argument('--llama-url', default='http://localhost:8080', help='llama.cpp server URL')
    
    args = parser.parse_args()
    
    controller = ScreenCaptureVLMWithDashboard(
        llama_server_url=args.llama_url,
        capture_fps=args.fps
    )
    
    controller.run()

if __name__ == "__main__":
    main()
