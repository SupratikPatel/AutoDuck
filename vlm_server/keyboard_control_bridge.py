#!/usr/bin/env python3
"""
VLM Keyboard Control Bridge for Duckiebot
Integrates VLM decision-making with keyboard control interface

This system:
1. Receives video feed from Duckiebot camera
2. Processes it through llamacpp VLM (Qwen2.5-VL)
3. Converts VLM decisions to keyboard inputs
4. Controls the Duckiebot through the existing keyboard GUI
"""

import cv2
import numpy as np
import requests
import base64
import time
import threading
import json
import asyncio
from datetime import datetime
from fastapi import FastAPI, WebSocket, HTTPException, BackgroundTasks
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from pydantic import BaseModel
from typing import Optional, Dict, List
import pyautogui
import keyboard
import platform
import subprocess
import os

# Platform-specific imports
if platform.system() == "Windows":
    import win32gui
    import win32con

class VideoFrame(BaseModel):
    """Model for incoming video frames"""
    image_base64: str
    timestamp: Optional[float] = None
    robot_name: Optional[str] = "duckiebot"

class VLMDecision(BaseModel):
    """Model for VLM decisions"""
    decision: str  # FORWARD, LEFT, RIGHT, STOP
    confidence: float
    reasoning: str
    timestamp: float

class ControlCommand(BaseModel):
    """Model for control commands"""
    command: str  # 'w', 'a', 's', 'd', 'space'
    duration: float  # seconds to hold the key
    robot_name: Optional[str] = "duckiebot"

class KeyboardControlBridge:
    def __init__(self, llama_server_url="http://localhost:8080", auto_control=True):
        self.llama_server_url = llama_server_url
        self.auto_control = auto_control
        self.current_decision = None
        self.processing = False
        self.control_active = False
        self.keyboard_gui_active = False
        
        # Performance tracking
        self.stats = {
            'total_frames': 0,
            'processed_frames': 0,
            'successful_decisions': 0,
            'failed_decisions': 0,
            'avg_response_time': 0.0,
            'current_fps': 0.0,
            'decision_history': []
        }
        
        # Key mapping for VLM decisions
        self.decision_to_key = {
            'FORWARD': 'w',     # or up arrow
            'LEFT': 'a',        # or left arrow
            'RIGHT': 'd',       # or right arrow
            'STOP': 's',        # or space
            'BACKWARD': 's'     # alternative stop
        }
        
        # Alternative arrow key mapping
        self.decision_to_arrow = {
            'FORWARD': 'up',
            'LEFT': 'left', 
            'RIGHT': 'right',
            'STOP': 'space'
        }
        
        # Control parameters
        self.key_press_duration = 0.1  # seconds
        self.decision_interval = 0.5    # seconds between decisions
        self.last_decision_time = 0
        
        # FastAPI app
        self.app = FastAPI(title="VLM Keyboard Control Bridge")
        self.setup_routes()
        
    def setup_routes(self):
        """Setup FastAPI routes"""
        # CORS middleware for cross-origin requests
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        @self.app.get("/")
        async def root():
            return HTMLResponse(content=self.get_dashboard_html())
        
        @self.app.post("/process_frame")
        async def process_frame(frame: VideoFrame):
            """Process a single video frame through VLM"""
            return await self.process_video_frame(frame)
        
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket for real-time video streaming"""
            await websocket.accept()
            try:
                while True:
                    data = await websocket.receive_json()
                    if 'image_base64' in data:
                        frame = VideoFrame(**data)
                        result = await self.process_video_frame(frame)
                        await websocket.send_json(result)
            except Exception as e:
                print(f"WebSocket error: {e}")
            finally:
                await websocket.close()
        
        @self.app.post("/control/manual")
        async def manual_control(command: ControlCommand):
            """Manual keyboard control endpoint"""
            self.send_keyboard_command(command.command, command.duration)
            return {"status": "ok", "command": command.command}
        
        @self.app.get("/control/status")
        async def control_status():
            """Get current control status"""
            return {
                "auto_control": self.auto_control,
                "control_active": self.control_active,
                "keyboard_gui_active": self.keyboard_gui_active,
                "current_decision": self.current_decision,
                "stats": self.stats
            }
        
        @self.app.post("/control/toggle")
        async def toggle_control():
            """Toggle automatic control on/off"""
            self.auto_control = not self.auto_control
            return {"auto_control": self.auto_control}
        
        @self.app.post("/control/activate_gui")
        async def activate_gui():
            """Activate the keyboard control GUI window"""
            self.activate_keyboard_gui()
            return {"status": "GUI activated"}
        
        @self.app.get("/stats")
        async def get_stats():
            """Get performance statistics"""
            return self.stats
    
    async def process_video_frame(self, frame: VideoFrame) -> Dict:
        """Process video frame through VLM and generate control decision"""
        self.stats['total_frames'] += 1
        
        # Rate limiting
        current_time = time.time()
        if current_time - self.last_decision_time < self.decision_interval:
            return {
                "status": "skipped",
                "reason": "rate_limited",
                "next_decision_in": self.decision_interval - (current_time - self.last_decision_time)
            }
        
        self.last_decision_time = current_time
        start_time = time.time()
        
        try:
            # Prepare VLM request
            vlm_request = {
                "model": "Qwen2.5-VL-7B",
                "messages": [{
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": self.get_driving_prompt()
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{frame.image_base64}"
                            }
                        }
                    ]
                }],
                "max_tokens": 50,
                "temperature": 0.3,
                "stream": False
            }
            
            # Send to llama.cpp server
            response = requests.post(
                f"{self.llama_server_url}/v1/chat/completions",
                json=vlm_request,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                content = result['choices'][0]['message']['content'].strip()
                
                # Parse decision
                decision = self.parse_vlm_decision(content)
                
                # Update stats
                processing_time = time.time() - start_time
                self.update_stats(decision, processing_time, success=True)
                
                # Execute control if auto mode is on
                if self.auto_control and self.keyboard_gui_active:
                    self.execute_decision(decision)
                
                self.current_decision = decision
                
                return {
                    "status": "success",
                    "decision": decision,
                    "reasoning": content,
                    "processing_time": processing_time,
                    "auto_executed": self.auto_control and self.keyboard_gui_active
                }
            else:
                raise Exception(f"VLM server error: {response.status_code}")
                
        except Exception as e:
            processing_time = time.time() - start_time
            self.update_stats(None, processing_time, success=False)
            return {
                "status": "error",
                "error": str(e),
                "processing_time": processing_time
            }
    
    def get_driving_prompt(self) -> str:
        """Get optimized prompt for autonomous driving"""
        return """You are controlling a Duckiebot through keyboard commands. Analyze this camera view and decide the best action.

Available commands:
- FORWARD: Move straight ahead (W key)
- LEFT: Turn left (A key) - use for lane changes or obstacle avoidance
- RIGHT: Turn right (D key) - use for lane changes or obstacle avoidance
- STOP: Stop immediately (S key)

Rules:
1. Stay in the lane (yellow line on left, white line on right)
2. Stop for obstacles, duckiebots, or red stop lines
3. DUCKIEBOT DETECTION: Look for blue/dark chassis with yellow wheels, yellow rubber duck on top, LED lights
4. RED STOP LINES: Stop for 5 seconds when red lines detected
5. LANE CHANGING: If obstacle ahead but adjacent lane clear (especially left lane), change lanes instead of stopping
6. Turn at intersections based on signs or lane markings
7. Prioritize safety - when uncertain, STOP

Navigation Strategy:
- Robot typically in right lane, so LEFT lane usually clearer for bypassing obstacles
- When duckiebot or obstacle in current lane, check if adjacent lane is clear for passing
- Execute lane change to avoid rather than just stopping when safe

Respond with just the command (FORWARD/LEFT/RIGHT/STOP) followed by brief reasoning.
Examples: 
- "FORWARD - Clear lane ahead, centered between lines"
- "LEFT - Duckiebot ahead, left lane clear, executing lane change"
- "STOP - Red stop line detected, stopping for 5 seconds" """
    
    def parse_vlm_decision(self, content: str) -> str:
        """Parse VLM response to extract decision"""
        content_upper = content.upper()
        
        # Look for decision keywords
        for decision in ['FORWARD', 'LEFT', 'RIGHT', 'STOP']:
            if decision in content_upper:
                return decision
        
        # Default to STOP for safety
        return 'STOP'
    
    def execute_decision(self, decision: str):
        """Execute keyboard command based on VLM decision"""
        if decision in self.decision_to_key:
            key = self.decision_to_key[decision]
            self.send_keyboard_command(key, self.key_press_duration)
    
    def send_keyboard_command(self, key: str, duration: float = 0.1):
        """Send keyboard input to control the robot"""
        try:
            # Ensure keyboard GUI is active
            if not self.keyboard_gui_active:
                self.activate_keyboard_gui()
                time.sleep(0.1)
            
            # Press and release key
            if key in ['up', 'down', 'left', 'right', 'space']:
                # Special keys
                pyautogui.keyDown(key)
                time.sleep(duration)
                pyautogui.keyUp(key)
            else:
                # Regular keys (w, a, s, d)
                pyautogui.keyDown(key)
                time.sleep(duration)
                pyautogui.keyUp(key)
                
            self.control_active = True
            
        except Exception as e:
            print(f"Keyboard control error: {e}")
            self.control_active = False
    
    def activate_keyboard_gui(self):
        """Activate the Duckiebot keyboard control GUI window"""
        try:
            if platform.system() == "Windows":
                # Find window by title on Windows
                def window_enum_handler(hwnd, windows):
                    if 'keyboard' in win32gui.GetWindowText(hwnd).lower():
                        windows.append(hwnd)
                
                windows = []
                win32gui.EnumWindows(window_enum_handler, windows)
                
                if windows:
                    win32gui.SetForegroundWindow(windows[0])
                    self.keyboard_gui_active = True
            else:
                # On Linux/Mac, use xdotool or similar
                # This is a placeholder - implement based on your system
                subprocess.run(["wmctrl", "-a", "keyboard"], check=False)
                self.keyboard_gui_active = True
                
        except Exception as e:
            print(f"Could not activate GUI window: {e}")
            self.keyboard_gui_active = False
    
    def update_stats(self, decision: Optional[str], processing_time: float, success: bool):
        """Update performance statistics"""
        if success:
            self.stats['processed_frames'] += 1
            self.stats['successful_decisions'] += 1
            
            # Update average response time
            total = self.stats['successful_decisions']
            self.stats['avg_response_time'] = (
                (self.stats['avg_response_time'] * (total - 1) + processing_time) / total
            )
            
            # Update FPS
            self.stats['current_fps'] = 1.0 / processing_time if processing_time > 0 else 0
            
            # Add to history
            self.stats['decision_history'].append({
                'timestamp': datetime.now().isoformat(),
                'decision': decision,
                'processing_time': processing_time
            })
            
            # Keep only last 100 decisions
            if len(self.stats['decision_history']) > 100:
                self.stats['decision_history'] = self.stats['decision_history'][-100:]
        else:
            self.stats['failed_decisions'] += 1
    
    def get_dashboard_html(self) -> str:
        """Get HTML for the control dashboard"""
        return """
<!DOCTYPE html>
<html>
<head>
    <title>VLM Keyboard Control Bridge</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #1a1a2e;
            color: white;
            margin: 0;
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
            padding: 20px;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            border-radius: 10px;
        }
        .grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        .card {
            background: #2d2d3d;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.3);
        }
        .control-panel {
            text-align: center;
        }
        .control-btn {
            background: #4CAF50;
            color: white;
            border: none;
            padding: 15px 30px;
            margin: 10px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 18px;
            transition: all 0.3s;
        }
        .control-btn:hover {
            background: #45a049;
            transform: scale(1.05);
        }
        .control-btn.stop {
            background: #f44336;
        }
        .control-btn.stop:hover {
            background: #da190b;
        }
        .stats {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }
        .stat-item {
            background: #3d3d4d;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }
        .stat-value {
            font-size: 24px;
            font-weight: bold;
            color: #4CAF50;
        }
        .decision-display {
            font-size: 48px;
            font-weight: bold;
            text-align: center;
            padding: 20px;
            border-radius: 10px;
            margin: 20px 0;
        }
        .decision-FORWARD { background: #4CAF50; }
        .decision-LEFT { background: #FF9800; }
        .decision-RIGHT { background: #2196F3; }
        .decision-STOP { background: #f44336; }
        .toggle-btn {
            background: #2196F3;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 16px;
        }
        .toggle-btn.active {
            background: #4CAF50;
        }
        #video-container {
            width: 100%;
            height: 400px;
            background: #000;
            border-radius: 10px;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .manual-controls {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 10px;
            max-width: 300px;
            margin: 20px auto;
        }
        .arrow-btn {
            width: 80px;
            height: 80px;
            font-size: 30px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 VLM Keyboard Control Bridge</h1>
            <p>Real-time Duckiebot control through Vision-Language Model</p>
        </div>
        
        <div class="grid">
            <div class="card">
                <h3>📹 Camera View</h3>
                <div id="video-container">
                    <canvas id="video-canvas" width="640" height="480"></canvas>
                </div>
                <div class="control-panel">
                    <button id="toggle-auto" class="toggle-btn" onclick="toggleAutoControl()">
                        Auto Control: OFF
                    </button>
                    <button class="control-btn" onclick="activateGUI()">
                        Activate Keyboard GUI
                    </button>
                </div>
            </div>
            
            <div class="card">
                <h3>🎮 Current Decision</h3>
                <div id="current-decision" class="decision-display">
                    WAITING...
                </div>
                <div id="reasoning" style="padding: 10px; background: #3d3d4d; border-radius: 8px; margin-top: 10px;">
                    Reasoning will appear here...
                </div>
                
                <h3 style="margin-top: 20px;">🕹️ Manual Controls</h3>
                <div class="manual-controls">
                    <div></div>
                    <button class="control-btn arrow-btn" onclick="sendCommand('w')">↑</button>
                    <div></div>
                    <button class="control-btn arrow-btn" onclick="sendCommand('a')">←</button>
                    <button class="control-btn arrow-btn stop" onclick="sendCommand('s')">⬛</button>
                    <button class="control-btn arrow-btn" onclick="sendCommand('d')">→</button>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h3>📊 Performance Statistics</h3>
            <div class="stats">
                <div class="stat-item">
                    <div class="stat-value" id="fps">0.00</div>
                    <div>Current FPS</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="avg-response">0.00s</div>
                    <div>Avg Response Time</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="processed">0</div>
                    <div>Processed Frames</div>
                </div>
                <div class="stat-item">
                    <div class="stat-value" id="success-rate">0%</div>
                    <div>Success Rate</div>
                </div>
            </div>
        </div>
        
        <div class="card">
            <h3>📜 Decision History</h3>
            <div id="decision-history" style="max-height: 200px; overflow-y: auto;">
                <!-- Decision history will be populated here -->
            </div>
        </div>
    </div>
    
    <script>
        let autoControl = false;
        let ws = null;
        
        // Initialize WebSocket connection
        function initWebSocket() {
            ws = new WebSocket(`ws://${window.location.host}/ws`);
            
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                updateUI(data);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
            
            ws.onclose = function() {
                setTimeout(initWebSocket, 3000); // Reconnect after 3 seconds
            };
        }
        
        // Update UI with new data
        function updateUI(data) {
            if (data.decision) {
                const decisionEl = document.getElementById('current-decision');
                decisionEl.textContent = data.decision;
                decisionEl.className = 'decision-display decision-' + data.decision;
                
                document.getElementById('reasoning').textContent = data.reasoning || '';
            }
            
            // Update stats periodically
            updateStats();
        }
        
        // Update statistics
        async function updateStats() {
            try {
                const response = await fetch('/stats');
                const stats = await response.json();
                
                document.getElementById('fps').textContent = stats.current_fps.toFixed(2);
                document.getElementById('avg-response').textContent = stats.avg_response_time.toFixed(2) + 's';
                document.getElementById('processed').textContent = stats.processed_frames;
                
                const successRate = stats.processed_frames > 0 
                    ? (stats.successful_decisions / stats.processed_frames * 100).toFixed(1)
                    : 0;
                document.getElementById('success-rate').textContent = successRate + '%';
                
                // Update decision history
                updateDecisionHistory(stats.decision_history);
            } catch (error) {
                console.error('Error updating stats:', error);
            }
        }
        
        // Update decision history display
        function updateDecisionHistory(history) {
            const historyEl = document.getElementById('decision-history');
            const recentDecisions = history.slice(-10).reverse();
            
            historyEl.innerHTML = recentDecisions.map(item => `
                <div style="padding: 5px; margin: 5px 0; background: #3d3d4d; border-radius: 5px;">
                    <strong>${item.decision}</strong> - ${new Date(item.timestamp).toLocaleTimeString()}
                    (${item.processing_time.toFixed(2)}s)
                </div>
            `).join('');
        }
        
        // Toggle auto control
        async function toggleAutoControl() {
            try {
                const response = await fetch('/control/toggle', { method: 'POST' });
                const data = await response.json();
                autoControl = data.auto_control;
                
                const btn = document.getElementById('toggle-auto');
                btn.textContent = `Auto Control: ${autoControl ? 'ON' : 'OFF'}`;
                btn.className = autoControl ? 'toggle-btn active' : 'toggle-btn';
            } catch (error) {
                console.error('Error toggling auto control:', error);
            }
        }
        
        // Send manual command
        async function sendCommand(command) {
            try {
                await fetch('/control/manual', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        command: command,
                        duration: 0.1
                    })
                });
            } catch (error) {
                console.error('Error sending command:', error);
            }
        }
        
        // Activate keyboard GUI
        async function activateGUI() {
            try {
                await fetch('/control/activate_gui', { method: 'POST' });
                alert('Keyboard GUI activated! Make sure the GUI window is visible.');
            } catch (error) {
                console.error('Error activating GUI:', error);
            }
        }
        
        // Initialize camera display (placeholder)
        function initCamera() {
            const canvas = document.getElementById('video-canvas');
            const ctx = canvas.getContext('2d');
            
            // Draw placeholder
            ctx.fillStyle = '#333';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = '#fff';
            ctx.font = '20px Arial';
            ctx.textAlign = 'center';
            ctx.fillText('Camera feed will appear here', canvas.width/2, canvas.height/2);
            ctx.fillText('when connected to Duckiebot', canvas.width/2, canvas.height/2 + 30);
        }
        
        // Initialize everything
        window.onload = function() {
            initWebSocket();
            initCamera();
            setInterval(updateStats, 1000);
        };
    </script>
</body>
</html>
"""

def create_app():
    """Create and configure the FastAPI app"""
    bridge = KeyboardControlBridge()
    return bridge.app

if __name__ == "__main__":
    print("🚀 Starting VLM Keyboard Control Bridge")
    print("📱 Dashboard: http://localhost:8000")
    print("🤖 Make sure llama.cpp server is running at http://localhost:8080")
    print("🎮 Launch Duckiebot keyboard control GUI with: dts duckiebot keyboard_control ROBOT_NAME")
    
    app = create_app()
    uvicorn.run(app, host="0.0.0.0", port=8000)
