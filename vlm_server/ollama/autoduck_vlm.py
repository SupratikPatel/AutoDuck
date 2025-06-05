#!/usr/bin/env python3
"""
Fast AutoDuck Real-time Webcam VLM System with Web Dashboard
Optimized for speed using direct llama.cpp server connection

Based on:
- https://github.com/ngxson/smolvlm-realtime-webcam  
- https://github.com/ggml-org/llama.cpp

This version uses direct HTTP calls to llama.cpp server for maximum speed
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

class FastAutoDuckVLM:
    def __init__(self):
        self.cap = None
        self.running = False
        self.continuous_mode = False
        self.processing = False
        self.last_result = None
        self.latest_frame = None
        self.request_logs = []
        
        # Performance settings for speed
        self.llama_cpp_url = "http://localhost:11434/api/generate"  # Direct Ollama API
        self.model_name = "gemma3:4b"
        
        # Speed optimizations
        self.image_quality = 60  # Lower quality for speed
        self.max_tokens = 20    # Allow for better reasoning
        self.temperature = 0.3  # Balanced deterministic/diverse
        
        # Statistics
        self.stats = {
            'frames_processed': 0,
            'successful_responses': 0,
            'avg_response_time': 0,
            'best_fps': 0,
            'session_start': time.time(),
            'decision_counts': {'forward': 0, 'left': 0, 'right': 0, 'stop': 0}
        }
        
        # Flask app for dashboard
        self.app = Flask(__name__)
        self.setup_routes()
        
    def log_request(self, request_type, url, status_code, response_time):
        """Log API requests for monitoring"""
        self.request_logs.append({
            'timestamp': datetime.now().strftime("%H:%M:%S.%f")[:-3],
            'type': request_type,
            'url': url,
            'status': status_code,
            'response_time': response_time
        })
        # Keep only last 50 logs
        if len(self.request_logs) > 50:
            self.request_logs = self.request_logs[-50:]
    
    def setup_routes(self):
        """Setup Flask routes for the dashboard"""
        
        @self.app.route('/')
        def dashboard():
            return render_template_string(DASHBOARD_TEMPLATE)
        
        @self.app.route('/api/latest_image')
        def latest_image():
            if self.latest_frame is not None:
                # Convert frame to JPEG
                _, buffer = cv2.imencode('.jpg', self.latest_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                return Response(buffer.tobytes(), mimetype='image/jpeg')
            return '', 404
        
        @self.app.route('/api/latest_analysis')
        def latest_analysis():
            if self.last_result:
                return jsonify(self.last_result)
            return jsonify({'decision': 'none', 'raw_response': 'waiting...', 'processing_time': 0})
        
        @self.app.route('/api/stats')
        def get_stats():
            runtime = time.time() - self.stats['session_start']
            return jsonify({
                **self.stats,
                'runtime_seconds': runtime,
                'runtime_formatted': f"{int(runtime//60)}m {int(runtime%60)}s"
            })
        
        @self.app.route('/api/request_logs')
        def get_request_logs():
            return jsonify(self.request_logs[-20:])  # Last 20 requests
        
        @self.app.route('/api/health')
        def health():
            return jsonify({'status': 'running', 'processing': self.processing})

    def initialize_camera(self):
        """Initialize camera with optimal settings for speed"""
        print("📹 Initializing high-speed camera...")
        
        for camera_id in [0, 1, 2]:
            self.cap = cv2.VideoCapture(camera_id)
            if self.cap.isOpened():
                ret, test_frame = self.cap.read()
                if ret and test_frame is not None:
                    # Balance between speed and full view
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # Full view resolution
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Full view resolution  
                    self.cap.set(cv2.CAP_PROP_FPS, 30)           # Good FPS
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)     # Minimize buffer lag
                    
                    print(f"✅ Fast camera {camera_id}: 640x480 @ 30fps")
                    return True
                else:
                    self.cap.release()
        
        print("❌ No camera found!")
        return False
    
    def test_llama_connection(self):
        """Test direct connection to llama.cpp/Ollama"""
        print("🔥 Testing llama.cpp connection...")
        
        try:
            # Test simple text generation for speed
            test_payload = {
                "model": self.model_name,
                "prompt": "Test: Respond with 'OK'",
                "stream": False,
                "options": {
                    "num_predict": 5,
                    "temperature": 0.1
                }
            }
            
            start_time = time.time()
            response = requests.post(self.llama_cpp_url, json=test_payload, timeout=10)
            test_time = time.time() - start_time
            
            self.log_request('POST', self.llama_cpp_url, response.status_code, test_time)
            
            if response.status_code == 200:
                result = response.json()
                test_response = result.get('response', '').strip()
                print(f"✅ llama.cpp connection: {test_time:.2f}s")
                print(f"✅ Test response: '{test_response}'")
                return True
            else:
                print(f"❌ llama.cpp error: {response.status_code}")
                return False
                
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def get_fast_prompt(self):
        """Ultra-optimized prompt for maximum speed"""
        return (
            "Autonomous robot car vision. Analyze road image for safe driving. "
            "STOP if people/obstacles ahead. LEFT/RIGHT for turns or avoidance. "
            "FORWARD only if completely clear path. "
            "Respond ONLY with: FORWARD, LEFT, RIGHT, or STOP."
        )
    
    def analyze_frame_fast(self, frame):
        """Super fast frame analysis using direct llama.cpp API"""
        if self.processing:
            return None
            
        try:
            self.processing = True
            start_time = time.time()
            
            # Store latest frame for dashboard
            self.latest_frame = frame.copy()
            
            # Encode with compression for speed
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.image_quality]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # Optimized payload
            payload = {
                "model": self.model_name,
                "prompt": f"<image>\n{self.get_fast_prompt()}",
                "images": [image_base64],
                "stream": False,
                "options": {
                    "num_predict": self.max_tokens,
                    "temperature": self.temperature,
                    "top_p": 0.9,
                    "repeat_penalty": 1.0
                }
            }
            
            # Direct HTTP call
            response = requests.post(self.llama_cpp_url, json=payload, timeout=15)
            processing_time = time.time() - start_time
            
            # Log the request
            self.log_request('POST', self.llama_cpp_url + ' (VLM)', response.status_code, processing_time)
            
            if response.status_code == 200:
                result = response.json()
                ai_response = result.get('response', '').strip()
                decision = self.parse_decision_fast(ai_response)
                
                # Update stats
                self.stats['frames_processed'] += 1
                self.stats['successful_responses'] += 1
                self.stats['decision_counts'][decision] += 1
                
                # Update average response time
                total_time = self.stats['avg_response_time'] * (self.stats['frames_processed'] - 1) + processing_time
                self.stats['avg_response_time'] = total_time / self.stats['frames_processed']
                
                current_fps = 1.0 / processing_time if processing_time > 0 else 0
                if current_fps > self.stats['best_fps']:
                    self.stats['best_fps'] = current_fps
                
                result = {
                    'decision': decision,
                    'raw_response': ai_response,
                    'processing_time': processing_time,
                    'fps': current_fps,
                    'timestamp': datetime.now().strftime("%H:%M:%S.%f")[:-3]
                }
                
                self.last_result = result
                print(f"🚗 {decision.upper()} | {processing_time:.2f}s | {current_fps:.2f} FPS | {ai_response[:20]}...")
                return result
            else:
                print(f"❌ API Error: {response.status_code}")
                return None
                
        except Exception as e:
            print(f"❌ Processing error: {e}")
            return None
        finally:
            self.processing = False
    
    def parse_decision_fast(self, response):
        """Fast decision parsing"""
        response_upper = response.upper()
        
        # Priority order for safety
        if 'STOP' in response_upper:
            return 'stop'
        elif 'FORWARD' in response_upper or 'AHEAD' in response_upper:
            return 'forward'
        elif 'LEFT' in response_upper:
            return 'left'
        elif 'RIGHT' in response_upper:
            return 'right'
        else:
            return 'stop'  # Default to safe
    
    def draw_fast_overlay(self, frame):
        """Minimal overlay for maximum performance"""
        h, w = frame.shape[:2]
        
        # Simple status bar
        cv2.rectangle(frame, (0, 0), (w, 60), (0, 0, 0), -1)
        
        # Title
        cv2.putText(frame, "AutoDuck Fast VLM", (10, 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Status
        if self.processing:
            cv2.putText(frame, "ANALYZING...", (250, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        else:
            cv2.putText(frame, "READY", (250, 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Last decision
        if self.last_result:
            decision = self.last_result['decision']
            colors = {
                'forward': (0, 255, 0),
                'left': (255, 255, 0), 
                'right': (255, 255, 0),
                'stop': (0, 0, 255)
            }
            color = colors.get(decision, (255, 255, 255))
            
            cv2.putText(frame, f"{decision.upper()}", (10, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(frame, f"{self.last_result['fps']:.2f} FPS", (150, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def continuous_analysis_worker(self):
        """Background worker for continuous mode"""
        last_analysis_time = 0
        analysis_interval = 1.0  # Analyze every 1 second in continuous mode
        
        while self.running and self.continuous_mode:
            current_time = time.time()
            
            if current_time - last_analysis_time >= analysis_interval and not self.processing:
                ret, frame = self.cap.read()
                if ret:
                    def analyze():
                        result = self.analyze_frame_fast(frame)
                        if result:
                            self.last_result = result
                    
                    threading.Thread(target=analyze, daemon=True).start()
                    last_analysis_time = current_time
            
            time.sleep(0.1)
    
    def run(self):
        """Main fast VLM loop"""
        print("🚀 AutoDuck Fast Real-time Webcam VLM")
        print("Optimized for speed using llama.cpp direct connection")
        print("=" * 60)
        
        # Test connection
        if not self.test_llama_connection():
            print("💡 Make sure Ollama is running: ollama serve")
            return
        
        # Initialize camera
        if not self.initialize_camera():
            return
        
        print(f"\n⚡ Speed optimizations active:")
        print(f"   📐 Resolution: 320x240 (fast)")
        print(f"   🖼️  JPEG Quality: {self.image_quality}% (fast)")
        print(f"   🎯 Max Tokens: {self.max_tokens} (fast)")
        print(f"   🌡️  Temperature: {self.temperature} (deterministic)")
        
        print(f"\n🎮 Controls:")
        print(f"   SPACE: Analyze current frame")
        print(f"   C: Toggle continuous mode")
        print(f"   Q: Quit")
        print(f"\n🚗 Starting fast analysis...")
        
        self.running = True
        continuous_worker = None
        
        try:
            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                # Mirror for user comfort
                frame = cv2.flip(frame, 1)
                
                # Draw minimal overlay
                frame_display = self.draw_fast_overlay(frame)
                
                # Show frame
                cv2.imshow("AutoDuck Fast VLM", frame_display)
                
                # Handle input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord(' ') and not self.processing:
                    # Manual analysis
                    def analyze():
                        result = self.analyze_frame_fast(frame)
                        if result:
                            self.last_result = result
                    threading.Thread(target=analyze, daemon=True).start()
                    
                elif key == ord('c'):
                    # Toggle continuous mode
                    self.continuous_mode = not self.continuous_mode
                    print(f"🔄 Continuous mode: {'ON' if self.continuous_mode else 'OFF'}")
                    
                    if self.continuous_mode and continuous_worker is None:
                        continuous_worker = threading.Thread(target=self.continuous_analysis_worker, daemon=True)
                        continuous_worker.start()
                
        except KeyboardInterrupt:
            print("\n⚠️  Interrupted")
        finally:
            self.running = False
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
            
            # Performance summary
            session_time = time.time() - self.stats['session_start']
            print(f"\n📊 Fast VLM Session Summary:")
            print(f"   ⏱️  Session Time: {session_time:.1f}s")
            print(f"   📷 Frames Processed: {self.stats['frames_processed']}")
            print(f"   ✅ Successful: {self.stats['successful_responses']}")
            if self.stats['frames_processed'] > 0:
                success_rate = (self.stats['successful_responses'] / self.stats['frames_processed']) * 100
                print(f"   📈 Success Rate: {success_rate:.1f}%")
            print(f"   ⚡ Average Response Time: {self.stats['avg_response_time']:.2f}s")
            print(f"   🏆 Best FPS: {self.stats['best_fps']:.2f}")
    
    def run_dashboard(self):
        """Run the web dashboard server"""
        print("🌐 Starting dashboard server at http://localhost:5000")
        self.app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

# Dashboard HTML Template
DASHBOARD_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AutoDuck VLM Dashboard</title>
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; padding: 20px; background: #1a1a1a; color: #fff; }
        .container { max-width: 1400px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 30px; }
        .header h1 { color: #00ff88; margin: 0; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; }
        .panel { background: #2a2a2a; border-radius: 10px; padding: 20px; border: 1px solid #444; }
        .image-panel { text-align: center; }
        .camera-feed { max-width: 100%; border-radius: 10px; border: 2px solid #555; }
        .analysis-panel { }
        .decision { font-size: 2em; font-weight: bold; margin: 10px 0; text-align: center; padding: 15px; border-radius: 10px; }
        .decision.forward { background: #2d5a2d; color: #90ee90; }
        .decision.left { background: #5a5a2d; color: #ffff90; }
        .decision.right { background: #5a5a2d; color: #ffff90; }
        .decision.stop { background: #5a2d2d; color: #ff9090; }
        .reasoning { background: #333; padding: 15px; border-radius: 5px; margin: 10px 0; font-family: monospace; }
        .stats { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
        .stat { background: #333; padding: 10px; border-radius: 5px; text-align: center; }
        .logs { background: #333; padding: 15px; border-radius: 5px; max-height: 200px; overflow-y: auto; font-family: monospace; font-size: 0.9em; }
        .log-entry { margin: 2px 0; padding: 2px; }
        .log-post { color: #90ee90; }
        .log-get { color: #90d0ff; }
        .decision-distribution { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin: 15px 0; }
        .decision-stat { text-align: center; background: #333; padding: 10px; border-radius: 5px; }
        .refresh-indicator { position: fixed; top: 10px; right: 10px; background: #00ff88; color: #000; padding: 5px 10px; border-radius: 5px; }
        @media (max-width: 768px) { .grid { grid-template-columns: 1fr; } }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚗 AutoDuck VLM Dashboard</h1>
            <p>Real-time Autonomous Driving Analysis</p>
        </div>
        
        <div class="grid">
            <div class="panel image-panel">
                <h3>📹 Camera Feed</h3>
                <img id="cameraFeed" class="camera-feed" src="/api/latest_image" alt="Camera feed loading...">
            </div>
            
            <div class="panel analysis-panel">
                <h3>🧠 VLM Analysis</h3>
                <div id="decision" class="decision">Waiting for analysis...</div>
                <div class="reasoning">
                    <strong>AI Response:</strong><br>
                    <span id="reasoning">Waiting for first analysis...</span>
                </div>
                <div class="reasoning">
                    <strong>Processing Time:</strong> <span id="processingTime">-</span>s<br>
                    <strong>FPS:</strong> <span id="fps">-</span><br>
                    <strong>Timestamp:</strong> <span id="timestamp">-</span>
                </div>
            </div>
        </div>
        
        <div class="grid" style="margin-top: 20px;">
            <div class="panel">
                <h3>📊 Performance Stats</h3>
                <div class="stats">
                    <div class="stat">
                        <strong>Session Runtime</strong><br>
                        <span id="runtime">-</span>
                    </div>
                    <div class="stat">
                        <strong>Frames Processed</strong><br>
                        <span id="framesProcessed">-</span>
                    </div>
                    <div class="stat">
                        <strong>Success Rate</strong><br>
                        <span id="successRate">-</span>%
                    </div>
                    <div class="stat">
                        <strong>Avg Response Time</strong><br>
                        <span id="avgResponseTime">-</span>s
                    </div>
                </div>
                
                <h4>Decision Distribution</h4>
                <div class="decision-distribution">
                    <div class="decision-stat">
                        <strong>FORWARD</strong><br>
                        <span id="forwardCount">0</span>
                    </div>
                    <div class="decision-stat">
                        <strong>LEFT</strong><br>
                        <span id="leftCount">0</span>
                    </div>
                    <div class="decision-stat">
                        <strong>RIGHT</strong><br>
                        <span id="rightCount">0</span>
                    </div>
                    <div class="decision-stat">
                        <strong>STOP</strong><br>
                        <span id="stopCount">0</span>
                    </div>
                </div>
            </div>
            
            <div class="panel">
                <h3>📡 Request Logs</h3>
                <div id="requestLogs" class="logs">Loading request logs...</div>
            </div>
        </div>
    </div>
    
    <div id="refreshIndicator" class="refresh-indicator" style="display: none;">↻</div>

    <script>
        let updateInterval;
        
        function updateDashboard() {
            const indicator = document.getElementById('refreshIndicator');
            indicator.style.display = 'block';
            
            // Update camera feed
            const img = document.getElementById('cameraFeed');
            img.src = '/api/latest_image?' + new Date().getTime();
            
            // Update analysis
            fetch('/api/latest_analysis')
                .then(response => response.json())
                .then(data => {
                    const decision = document.getElementById('decision');
                    decision.textContent = data.decision.toUpperCase();
                    decision.className = 'decision ' + data.decision;
                    
                    document.getElementById('reasoning').textContent = data.raw_response || 'No response yet';
                    document.getElementById('processingTime').textContent = data.processing_time?.toFixed(2) || '-';
                    document.getElementById('fps').textContent = data.fps?.toFixed(2) || '-';
                    document.getElementById('timestamp').textContent = data.timestamp || '-';
                });
            
            // Update stats
            fetch('/api/stats')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('runtime').textContent = data.runtime_formatted || '-';
                    document.getElementById('framesProcessed').textContent = data.frames_processed || 0;
                    
                    const successRate = data.frames_processed > 0 ? 
                        ((data.successful_responses / data.frames_processed) * 100).toFixed(1) : 0;
                    document.getElementById('successRate').textContent = successRate;
                    
                    document.getElementById('avgResponseTime').textContent = data.avg_response_time?.toFixed(2) || '-';
                    
                    // Decision counts
                    document.getElementById('forwardCount').textContent = data.decision_counts?.forward || 0;
                    document.getElementById('leftCount').textContent = data.decision_counts?.left || 0;
                    document.getElementById('rightCount').textContent = data.decision_counts?.right || 0;
                    document.getElementById('stopCount').textContent = data.decision_counts?.stop || 0;
                });
            
            // Update request logs
            fetch('/api/request_logs')
                .then(response => response.json())
                .then(data => {
                    const logs = document.getElementById('requestLogs');
                    logs.innerHTML = data.map(log => 
                        `<div class="log-entry log-${log.type.toLowerCase()}">
                            [${log.timestamp}] ${log.type} ${log.url} - ${log.status} (${log.response_time.toFixed(3)}s)
                        </div>`
                    ).reverse().join('');
                });
            
            setTimeout(() => {
                indicator.style.display = 'none';
            }, 200);
        }
        
        // Initial load
        updateDashboard();
        
        // Auto-refresh every 2 seconds
        updateInterval = setInterval(updateDashboard, 2000);
        
        // Refresh on page focus
        window.addEventListener('focus', updateDashboard);
    </script>
</body>
</html>
"""

def main():
    """Entry point for fast VLM system"""
    import sys
    fast_vlm = FastAutoDuckVLM()
    
    if len(sys.argv) > 1 and sys.argv[1] == '--dashboard-only':
        # Run only the dashboard server
        fast_vlm.run_dashboard()
    else:
        # Run camera interface and dashboard server in separate threads
        print("🚀 Starting AutoDuck VLM with Dashboard")
        print("🌐 Dashboard: http://localhost:5000")
        
        # Start dashboard in background
        dashboard_thread = threading.Thread(target=fast_vlm.run_dashboard, daemon=True)
        dashboard_thread.start()
        
        # Start main VLM interface
        fast_vlm.run()

if __name__ == "__main__":
    main() 