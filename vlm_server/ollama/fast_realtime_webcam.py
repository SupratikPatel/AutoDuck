#!/usr/bin/env python3
"""
Fast AutoDuck Real-time Webcam VLM System
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

class FastAutoDuckVLM:
    def __init__(self):
        self.cap = None
        self.running = False
        self.continuous_mode = False
        self.processing = False
        self.last_result = None
        
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
            'session_start': time.time()
        }
        
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
            
            if response.status_code == 200:
                result = response.json()
                ai_response = result.get('response', '').strip()
                decision = self.parse_decision_fast(ai_response)
                
                # Update stats
                self.stats['frames_processed'] += 1
                self.stats['successful_responses'] += 1
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

def main():
    """Entry point for fast VLM system"""
    fast_vlm = FastAutoDuckVLM()
    fast_vlm.run()

if __name__ == "__main__":
    main() 