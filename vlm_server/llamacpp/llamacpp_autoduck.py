#!/usr/bin/env python3
"""
Ultra-Fast AutoDuck VLM System with llama.cpp + CUDA
Connects to native llama.cpp server for maximum speed performance

Performance Target: <1 second response time for real-time robot control
"""

import cv2
import numpy as np
import requests
import base64
import time
import threading
import json
from datetime import datetime
from flask import Flask, render_template_string, jsonify, Response
import io

class LlamaCppAutoDuckVLM:
    def __init__(self, server_url="http://localhost:8080"):
        self.server_url = server_url
        self.cap = None
        self.running = False
        self.continuous_mode = False
        self.processing = False
        self.last_result = None
        self.last_image = None
        self.last_analysis_time = None
        
        # Enhanced navigation with decision memory
        self.last_decisions = []  # Track recent decisions for anti-oscillation
        self.consecutive_stops = 0  # Count consecutive STOP decisions
        self.obstacle_avoidance_mode = False
        self.stats_printed_at = 0  # Track when stats were last printed
        
        # Performance tracking
        self.stats = {
            'total_requests': 0,
            'successful_requests': 0,
            'failed_requests': 0,
            'avg_response_time': 0.0,
            'best_response_time': float('inf'),
            'worst_response_time': 0.0,
            'current_fps': 0.0,
            'best_fps': 0.0,
            'decisions': {'FORWARD': 0, 'LEFT': 0, 'RIGHT': 0, 'STOP': 0}
        }
        
        # Response times for averaging
        self.response_times = []
        
        # Flask dashboard
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Speed optimizations for llama.cpp
        self.image_quality = 85    # Higher quality for better analysis
        self.max_tokens = 50       # Allow more reasoning
        self.temperature = 0.4     # Balanced creativity/consistency

    def setup_routes(self):
        """Setup Flask routes for dashboard"""
        @self.app.route('/')
        def dashboard():
            return render_template_string(DASHBOARD_TEMPLATE)
        
        @self.app.route('/api/stats')
        def get_stats():
            return jsonify(self.stats)
        
        @self.app.route('/api/latest')
        def get_latest():
            if self.last_result:
                return jsonify({
                    'result': self.last_result,
                    'timestamp': self.last_analysis_time,
                    'processing': self.processing
                })
            return jsonify({'result': None})
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(self.generate_frames(), 
                          mimetype='multipart/x-mixed-replace; boundary=frame')

    def generate_frames(self):
        """Generate video frames for dashboard"""
        while True:
            if self.last_image is not None:
                # Add overlay with current decision
                frame = self.last_image.copy()
                self.add_overlay(frame)
                
                # Encode frame
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.1)

    def setup_camera(self, camera_id=0):
        """Initialize camera with optimized settings"""
        print(f"🎥 Setting up camera {camera_id}...")
        
        # Release any existing camera
        if self.cap:
            self.cap.release()
        
        # Try different camera IDs
        for cid in [camera_id, 0, 1, 2]:
            try:
                self.cap = cv2.VideoCapture(cid)
                if self.cap.isOpened():
                    # Optimized settings for balance of speed and quality
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Good resolution
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Good resolution  
                    self.cap.set(cv2.CAP_PROP_FPS, 30)           # High FPS
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # Minimize lag
                    
                    # Test capture
                    ret, frame = self.cap.read()
                    if ret:
                        print(f"✅ Camera {cid} ready: 640x480 @ 30fps")
                        return True
                        
            except Exception as e:
                print(f"❌ Camera {cid} failed: {e}")
                continue
        
        print("❌ No working camera found!")
        return False

    def check_server_connection(self):
        """Check if llama.cpp server is running"""
        try:
            response = requests.get(f"{self.server_url}/health", timeout=5)
            if response.status_code == 200:
                print(f"✅ llama.cpp server is running at {self.server_url}")
                return True
        except Exception as e:
            print(f"❌ Cannot connect to llama.cpp server: {e}")
            print(f"   Make sure server is running:")
            print(f"   docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \\")
            print(f"       -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \\")
            print(f"       --host 0.0.0.0 --port 8080 \\")
            print(f"       --n-gpu-layers 99 --ctx-size 1024 --batch-size 256 --threads 4 --cont-batching")
            print(f"   docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \\")
            print(f"       -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \\")
            print(f"       --host 0.0.0.0 --port 8080 --n-gpu-layers 99")
            return False
        return False

    def get_autonomous_prompt(self):
        """Enhanced prompt for autonomous driving decisions with Duckiebot and duck detection"""
        return """You are controlling a small, slow-moving Duckiebot robot car in a miniature Duckietown environment. Analyze this camera view and decide the best driving action.

CRITICAL DETECTION REQUIREMENTS:
- Objects that appear large may actually be FAR AWAY due to the small scale environment
- Only STOP when obstacles are VERY CLOSE (within 20cm or filling most of the camera view)
- You can continue FORWARD even with obstacles visible ahead if they are distant

Available commands:
- FORWARD: Continue straight ahead (preferred action for clear or distant obstacles)
- LEFT: Turn left (when path ahead is blocked and left side is clear)
- RIGHT: Turn right (when path ahead is blocked and right side is clear)  
- STOP: Stop for extremely close obstacles, immediate danger, or Duckietown inhabitants within 20cm

🦆 MANDATORY STOP CONDITIONS (within 20cm):
1. DUCKIEBOTS: Small robotic vehicles with these distinctive features:
   * Blue/dark blue/black main chassis (rectangular robot body)
   * Bright yellow wheels on sides
   * Yellow rubber duck mounted on top (signature duckiebot feature)
   * LED strip lights (often colored/blinking) 
   * Visible electronic components, cameras, or sensors
   * Approximately 10-15cm in size
   * STOP immediately if within 20cm, wait for them to pass

2. YELLOW RUBBER DUCKS: Small yellow duck toys or any duck-like objects
   * Classic yellow rubber duck shape
   * Any small yellow duck toys on the road or nearby
   * STOP immediately if within 20cm to protect Duckietown inhabitants

3. RED LINES: Red stop lines or red tape markings
   * Red colored lines across the road surface
   * Red tape or paint markings indicating stop zones
   * STOP when within 20cm and wait before proceeding

Navigation Strategy:
- DEFAULT to FORWARD unless path is completely blocked
- Objects in distance = FORWARD (they're farther than they look due to miniature scale)
- 🦆 SAFETY OVERRIDE: Always STOP for duckiebots, yellow ducks, or red lines within 20cm

🛣️ OBSTACLE AVOIDANCE PROTOCOL:
When encountering obstacles (duckiebots, yellow ducks, red lines, or other barriers):
1. FIRST: STOP immediately when obstacle is within 20cm
2. THEN: Execute LEFT command to steer into left vacant lane to go around obstacle
3. NAVIGATE: Execute RIGHT command to go around the obstacle when:
   - White lane lines are too close (indicating edge of left lane), OR
   - Need to navigate around the obstacle to avoid collision, OR
   - After brief left steering to position for going around
4. RETURN: Execute RIGHT command to return to original right lane when:
   - Obstacle is no longer visible/detected, OR
   - After approximately 3 seconds of maneuvering
5. RESUME: Continue FORWARD once back in proper lane

- This lane-changing strategy ensures safe obstacle avoidance while maintaining traffic flow
- Use RIGHT command to navigate around obstacles after initial LEFT positioning
- Return to right lane as soon as obstacle is cleared or after 3 seconds maximum
- Only use LEFT/RIGHT for strategic lane changes, not for simple turns
- Balance forward progress with Duckietown safety - protect the ducks and robots!

Respond with: COMMAND - Brief reasoning

Examples:
- "FORWARD - Road clear, obstacles distant, no ducks or robots nearby"
- "STOP - Yellow duck toy detected within 20cm, protecting Duckietown inhabitant - will execute left lane change"
- "STOP - Duckiebot detected: blue chassis with yellow wheels and rubber duck on top, within 20cm - stopping then left lane change"
- "STOP - Red stop line within 20cm, stopping as required - will use left lane to bypass"
- "LEFT - Executing lane change to go around obstacle, steering into left vacant lane"
- "RIGHT - White lane lines too close, navigating around obstacle to avoid collision"
- "RIGHT - Going around obstacle after left positioning, steering right to bypass"
- "RIGHT - Obstacle no longer visible, returning to original right lane immediately"
- "RIGHT - After 3 seconds of maneuvering, returning to original right lane"
- "FORWARD - Back in proper lane, continuing forward progress" """

    def encode_image_for_llamacpp(self, frame):
        """Encode image for llama.cpp server"""
        # Resize for optimal processing speed
        height, width = frame.shape[:2]
        if width > 640:
            scale = 640.0 / width
            new_width = 640
            new_height = int(height * scale)
            frame = cv2.resize(frame, (new_width, new_height))
        
        # Encode as JPEG with balanced quality
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]
        result, encimg = cv2.imencode('.jpg', frame, encode_param)
        
        if result:
            # Convert to base64
            img_base64 = base64.b64encode(encimg).decode('utf-8')
            return f"data:image/jpeg;base64,{img_base64}"
        return None

    def analyze_frame(self, frame):
        """Send frame to llama.cpp server for analysis"""
        if frame is None:
            return None
            
        try:
            start_time = time.time()
            
            # Encode image
            image_data = self.encode_image_for_llamacpp(frame)
            if not image_data:
                return None
            
            # Prepare request for llama.cpp OpenAI-compatible API
            payload = {
                "model": "gemma-3-4b-it",
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": self.get_autonomous_prompt()},
                            {"type": "image_url", "image_url": {"url": image_data}}
                        ]
                    }
                ],
                "max_tokens": self.max_tokens,
                "temperature": self.temperature,
                "stream": False
            }
            
            # Send request
            response = requests.post(
                f"{self.server_url}/v1/chat/completions",
                json=payload,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            end_time = time.time()
            response_time = end_time - start_time
            
            if response.status_code == 200:
                result = response.json()
                content = result['choices'][0]['message']['content'].strip()
                
                # Parse decision
                decision = self.parse_decision(content)
                
                # Update stats
                self.update_stats(response_time, decision, True)
                
                return {
                    'decision': decision,
                    'reasoning': content,
                    'response_time': response_time,
                    'timestamp': datetime.now().isoformat()
                }
            else:
                print(f"❌ Server error: {response.status_code} - {response.text}")
                self.update_stats(response_time, None, False)
                return None
                
        except Exception as e:
            print(f"❌ Analysis error: {e}")
            self.update_stats(0, None, False)
            return None

    def parse_decision(self, content):
        """Extract decision from AI response with anti-deadlock logic"""
        content_upper = content.upper()
        
        # Look for decision words
        raw_decision = None
        if 'STOP' in content_upper:
            raw_decision = 'STOP'
        elif 'LEFT' in content_upper:
            raw_decision = 'LEFT'
        elif 'RIGHT' in content_upper:
            raw_decision = 'RIGHT'
        elif 'FORWARD' in content_upper:
            raw_decision = 'FORWARD'
        else:
            # Default to STOP for safety
            raw_decision = 'STOP'
        
        # Apply anti-deadlock logic
        final_decision = self.apply_anti_deadlock_logic(raw_decision)
        
        # Update decision history
        self.last_decisions.append(final_decision)
        if len(self.last_decisions) > 5:  # Keep last 5 decisions
            self.last_decisions = self.last_decisions[-5:]
        
        return final_decision

    def apply_anti_deadlock_logic(self, decision):
        """Prevent getting stuck with smart navigation logic"""
        # Count consecutive STOPs
        if decision == 'STOP':
            self.consecutive_stops += 1
        else:
            self.consecutive_stops = 0
        
        # If too many consecutive STOPs, force alternative navigation
        if self.consecutive_stops >= 3:
            print(f"🚨 Anti-deadlock: {self.consecutive_stops} consecutive STOPs detected!")
            
            # Analyze recent decisions to choose best alternative
            if len(self.last_decisions) >= 2:
                recent = self.last_decisions[-2:]
                # Avoid oscillation between LEFT and RIGHT
                if 'LEFT' in recent and 'RIGHT' not in recent:
                    print("   → Forcing RIGHT to explore new path")
                    self.consecutive_stops = 0
                    return 'RIGHT'
                elif 'RIGHT' in recent and 'LEFT' not in recent:
                    print("   → Forcing LEFT to explore new path")
                    self.consecutive_stops = 0
                    return 'LEFT'
            
            # Default: try RIGHT first, then LEFT
            if self.consecutive_stops % 2 == 1:
                print("   → Forcing RIGHT to bypass obstacle")
                return 'RIGHT'
            else:
                print("   → Forcing LEFT to bypass obstacle")
                return 'LEFT'
        
        # Check for LEFT-RIGHT oscillation
        if len(self.last_decisions) >= 4:
            recent_4 = self.last_decisions[-4:]
            left_count = recent_4.count('LEFT')
            right_count = recent_4.count('RIGHT')
            
            # If oscillating between LEFT and RIGHT, try FORWARD
            if left_count >= 2 and right_count >= 2:
                print("🔄 Anti-oscillation: Detected LEFT-RIGHT oscillation, trying FORWARD")
                return 'FORWARD'
        
        return decision

    def update_stats(self, response_time, decision, success):
        """Update performance statistics"""
        self.stats['total_requests'] += 1
        
        if success:
            self.stats['successful_requests'] += 1
            
            # Update response times
            self.response_times.append(response_time)
            if len(self.response_times) > 100:  # Keep last 100
                self.response_times = self.response_times[-100:]
            
            self.stats['avg_response_time'] = sum(self.response_times) / len(self.response_times)
            self.stats['best_response_time'] = min(self.stats['best_response_time'], response_time)
            self.stats['worst_response_time'] = max(self.stats['worst_response_time'], response_time)
            
            # Update FPS
            current_fps = 1.0 / response_time if response_time > 0 else 0
            self.stats['current_fps'] = current_fps
            self.stats['best_fps'] = max(self.stats['best_fps'], current_fps)
            
            # Update decision counts
            if decision in self.stats['decisions']:
                self.stats['decisions'][decision] += 1
        else:
            self.stats['failed_requests'] += 1

    def add_overlay(self, frame):
        """Add performance overlay to frame"""
        if not self.last_result:
            return
        
        # Overlay background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (300, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Decision text
        decision = self.last_result.get('decision', 'UNKNOWN')
        color = {
            'FORWARD': (0, 255, 0),    # Green
            'LEFT': (0, 255, 255),     # Yellow  
            'RIGHT': (255, 0, 255),    # Magenta
            'STOP': (0, 0, 255),       # Red
        }.get(decision, (255, 255, 255))
        
        cv2.putText(frame, f"Decision: {decision}", (20, 35), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Performance stats
        rt = self.last_result.get('response_time', 0)
        fps = 1.0 / rt if rt > 0 else 0
        
        cv2.putText(frame, f"Response: {rt:.2f}s", (20, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"FPS: {fps:.2f}", (20, 80), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Total: {self.stats['total_requests']}", (20, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def run(self):
        """Main execution loop"""
        print("🚀 Starting Ultra-Fast AutoDuck VLM with llama.cpp")
        
        # Check server connection
        if not self.check_server_connection():
            print("⚠️  Please start the llama.cpp server first!")
            return
        
        # Setup camera
        if not self.setup_camera():
            print("❌ Cannot setup camera!")
            return
        
        # Start dashboard in background
        dashboard_thread = threading.Thread(target=self.run_dashboard, daemon=True)
        dashboard_thread.start()
        
        print("\n🎮 Controls:")
        print("  SPACE: Single frame analysis")
        print("  C: Toggle continuous mode")
        print("  Q: Quit")
        print(f"  Dashboard: http://localhost:5000")
        
        self.running = True
        
        try:
            while self.running:
                # Capture frame
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ Failed to capture frame")
                    break
                
                self.last_image = frame.copy()
                
                # Handle user input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    self.continuous_mode = not self.continuous_mode
                    print(f"📱 Continuous mode: {'ON' if self.continuous_mode else 'OFF'}")
                elif key == ord(' ') or self.continuous_mode:
                    if not self.processing:
                        # Start analysis in background
                        analysis_thread = threading.Thread(
                            target=self.process_frame_async, 
                            args=(frame.copy(),), 
                            daemon=True
                        )
                        analysis_thread.start()
                
                # Add overlay and display
                display_frame = frame.copy()
                self.add_overlay(display_frame)
                cv2.imshow('Ultra-Fast AutoDuck VLM', display_frame)
                
                # Print stats every 10 successful requests (but only once per milestone)
                if (self.stats['successful_requests'] > 0 and 
                    self.stats['successful_requests'] % 10 == 0 and
                    self.stats['successful_requests'] > self.stats_printed_at):
                    self.print_stats()
                    self.stats_printed_at = self.stats['successful_requests']
                    
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    def process_frame_async(self, frame):
        """Process frame asynchronously"""
        self.processing = True
        result = self.analyze_frame(frame)
        if result:
            self.last_result = result
            self.last_analysis_time = result['timestamp']
        self.processing = False

    def cleanup(self):
        """Clean up resources and show final summary"""
        print("\n🛑 Shutting down...")
        self.running = False
        
        # Show final comprehensive summary
        self.print_final_summary()
        
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

    def print_stats(self):
        """Print current performance statistics"""
        print(f"\n📊 Performance Checkpoint (Request #{self.stats['successful_requests']}):")
        print(f"   ✅ Success Rate: {self.stats['successful_requests']}/{self.stats['total_requests']} ({100*self.stats['successful_requests']/max(1,self.stats['total_requests']):.1f}%)")
        print(f"   ⚡ Avg Response: {self.stats['avg_response_time']:.2f}s")
        print(f"   📈 Current FPS: {self.stats['current_fps']:.2f}")
        print(f"   🎮 Recent Decisions: {self.last_decisions[-3:] if len(self.last_decisions) >= 3 else self.last_decisions}")

    def print_final_summary(self):
        """Print comprehensive final summary when quitting"""
        print("\n" + "="*60)
        print("🏁 FINAL AUTODUCK VLM SESSION SUMMARY")
        print("="*60)
        
        # Performance Overview
        print(f"\n📊 PERFORMANCE OVERVIEW:")
        print(f"   🎯 Total Requests: {self.stats['total_requests']}")
        print(f"   ✅ Successful: {self.stats['successful_requests']}")
        print(f"   ❌ Failed: {self.stats['failed_requests']}")
        success_rate = 100*self.stats['successful_requests']/max(1,self.stats['total_requests'])
        print(f"   📈 Success Rate: {success_rate:.1f}%")
        
        # Response Time Analysis
        print(f"\n⚡ RESPONSE TIME ANALYSIS:")
        print(f"   🎯 Average: {self.stats['avg_response_time']:.2f}s")
        print(f"   🏆 Best: {self.stats['best_response_time']:.2f}s")
        print(f"   📉 Worst: {self.stats['worst_response_time']:.2f}s")
        print(f"   🚀 Best FPS: {self.stats['best_fps']:.2f}")
        
        # Decision Analysis
        total_decisions = sum(self.stats['decisions'].values())
        print(f"\n🎮 NAVIGATION DECISION ANALYSIS:")
        for decision, count in self.stats['decisions'].items():
            percentage = 100 * count / max(1, total_decisions)
            print(f"   {decision}: {count} ({percentage:.1f}%)")
        
        # Navigation Quality Assessment
        print(f"\n🧭 NAVIGATION QUALITY ASSESSMENT:")
        stop_percentage = 100 * self.stats['decisions']['STOP'] / max(1, total_decisions)
        if stop_percentage > 60:
            print(f"   ⚠️  High STOP rate ({stop_percentage:.1f}%) - Consider obstacle avoidance improvements")
        elif stop_percentage > 30:
            print(f"   ⚡ Moderate STOP rate ({stop_percentage:.1f}%) - Good safety balance")
        else:
            print(f"   🎯 Low STOP rate ({stop_percentage:.1f}%) - Excellent navigation flow")
        
        # Anti-deadlock Performance
        forward_percentage = 100 * self.stats['decisions']['FORWARD'] / max(1, total_decisions)
        maneuver_percentage = 100 * (self.stats['decisions']['LEFT'] + self.stats['decisions']['RIGHT']) / max(1, total_decisions)
        print(f"   🚗 Movement: {forward_percentage:.1f}% forward, {maneuver_percentage:.1f}% maneuvering")
        
        # Recent Decision Pattern
        if len(self.last_decisions) >= 5:
            print(f"   🔄 Final 5 decisions: {' → '.join(self.last_decisions[-5:])}")
        
        # Performance Rating
        avg_fps = 1.0 / self.stats['avg_response_time'] if self.stats['avg_response_time'] > 0 else 0
        print(f"\n🏆 OVERALL PERFORMANCE RATING:")
        if avg_fps >= 1.5:
            print(f"   🥇 EXCELLENT (>{avg_fps:.1f} FPS) - Real-time capable!")
        elif avg_fps >= 1.0:
            print(f"   🥈 GOOD ({avg_fps:.1f} FPS) - Near real-time performance")
        elif avg_fps >= 0.5:
            print(f"   🥉 FAIR ({avg_fps:.1f} FPS) - Acceptable for testing")
        else:
            print(f"   ⚠️  NEEDS IMPROVEMENT ({avg_fps:.1f} FPS) - Consider optimization")
        
        print("\n" + "="*60)
        print("Thank you for using AutoDuck VLM! 🦆🤖")
        print("="*60)

    def run_dashboard(self):
        """Run the web dashboard server"""
        print("🌐 Dashboard starting at http://localhost:5000")
        self.app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

# Dashboard HTML Template
DASHBOARD_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Ultra-Fast AutoDuck VLM - llama.cpp Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #1e1e1e; color: white; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 30px; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .card { background: #2d2d2d; border-radius: 10px; padding: 20px; }
        .video-container { position: relative; }
        .video-feed { width: 100%; border-radius: 10px; }
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
        .performance-chart { height: 200px; background: #3d3d3d; border-radius: 8px; margin-top: 15px; }
        .loading { animation: pulse 1s infinite; }
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.5; } 100% { opacity: 1; } }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚀 Ultra-Fast AutoDuck VLM</h1>
            <p>Real-time Autonomous Driving with llama.cpp + CUDA</p>
        </div>
        
        <div class="grid">
            <div class="card">
                <h3>📹 Live Camera Feed</h3>
                <div class="video-container">
                    <img class="video-feed" src="/video_feed" alt="Camera Feed">
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
        
        <div class="card" style="margin-top: 20px;">
            <h3>📊 Performance Metrics</h3>
            <div class="stats-grid">
                <div class="stat-item">
                    <div id="total-requests" class="stat-number">0</div>
                    <div class="stat-label">Total Requests</div>
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
                <div class="stat-item">
                    <div id="best-response" class="stat-number">∞</div>
                    <div class="stat-label">Best Response Time</div>
                </div>
                <div class="stat-item">
                    <div id="best-fps" class="stat-number">0.00</div>
                    <div class="stat-label">Best FPS</div>
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
    </div>

    <script>
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
                });
            
            // Update stats
            fetch('/api/stats')
                .then(response => response.json())
                .then(stats => {
                    document.getElementById('total-requests').textContent = stats.total_requests;
                    
                    const successRate = stats.total_requests > 0 ? 
                        (100 * stats.successful_requests / stats.total_requests).toFixed(1) : 0;
                    document.getElementById('success-rate').textContent = successRate + '%';
                    
                    document.getElementById('avg-response').textContent = stats.avg_response_time.toFixed(2) + 's';
                    document.getElementById('current-fps').textContent = stats.current_fps.toFixed(2);
                    document.getElementById('best-response').textContent = 
                        stats.best_response_time === Infinity ? '∞' : stats.best_response_time.toFixed(2) + 's';
                    document.getElementById('best-fps').textContent = stats.best_fps.toFixed(2);
                    
                    // Decision counts
                    document.getElementById('forward-count').textContent = stats.decisions.FORWARD;
                    document.getElementById('left-count').textContent = stats.decisions.LEFT;
                    document.getElementById('right-count').textContent = stats.decisions.RIGHT;
                    document.getElementById('stop-count').textContent = stats.decisions.STOP;
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
    """Entry point for ultra-fast llama.cpp VLM system"""
    llamacpp_vlm = LlamaCppAutoDuckVLM()
    llamacpp_vlm.run()

if __name__ == "__main__":
    main() 