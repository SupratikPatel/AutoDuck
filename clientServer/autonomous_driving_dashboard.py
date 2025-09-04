#!/usr/bin/env python3
"""
Autonomous Driving Dashboard
Web dashboard for monitoring DuckieBot autonomous driving system
Similar to screen_capture_vlm_with_dashboard.py but for autonomous driving
"""

import cv2
import numpy as np
import requests
import base64
import time
import threading
import json
import logging
from datetime import datetime
from flask import Flask, render_template_string, jsonify, Response
import io

class AutonomousDrivingDashboard:
    def __init__(self, server_url="http://192.168.140.179:8000"):
        self.server_url = server_url

        # Dashboard state
        self.last_analysis = None
        self.last_image = None
        self.stats = {
            'total_requests': 0,
            'successful_analyses': 0,
            'failed_analyses': 0,
            'avg_response_time': 0.0,
            'emergency_stops': 0,
            'lane_following_confidence': 0.0,
            'current_state': 'unknown',
            'duck_emergency_stops': 0,
            'duck_detection_events': 0
        }

        # Control settings
        self.auto_mode = True
        self.emergency_stop_override = False

        # Duck emergency stop tracking
        self.duck_emergency_stop_active = False
        self.duck_detection_count = 0

        # Flask app
        self.app = Flask(__name__)
        self.setup_routes()

        print("🚗 Autonomous Driving Dashboard")
        print(f"📊 Dashboard will be available at: http://localhost:4000")
        print(f"🧠 Server should be running at: {server_url}")

    def setup_routes(self):
        """Setup Flask routes for the dashboard"""

        @self.app.route('/')
        def dashboard():
            return render_template_string(DASHBOARD_HTML)

        @self.app.route('/api/stats')
        def get_stats():
            return jsonify(self.stats)

        @self.app.route('/api/latest')
        def get_latest():
            if self.last_analysis:
                return jsonify({
                    'analysis': self.last_analysis,
                    'timestamp': datetime.now().isoformat(),
                    'auto_mode': self.auto_mode,
                    'emergency_override': self.emergency_stop_override,
                    'duck_emergency_stop': self.duck_emergency_stop_active,
                    'duck_detection_count': self.duck_detection_count
                })
            return jsonify({'analysis': None})

        @self.app.route('/api/toggle_auto')
        def toggle_auto():
            self.auto_mode = not self.auto_mode
            return jsonify({'auto_mode': self.auto_mode})

        @self.app.route('/api/emergency_stop')
        def emergency_stop():
            self.emergency_stop_override = True
            self.stats['emergency_stops'] += 1
            return jsonify({'emergency_stop': True})

        @self.app.route('/api/resume')
        def resume():
            self.emergency_stop_override = False
            return jsonify({'emergency_stop': False})

        @self.app.route('/api/test_analysis')
        def test_analysis():
            self.test_server_connection()
            return jsonify({'status': 'tested'})

    def test_server_connection(self):
        """Test connection to autonomous driving server"""
        try:
            # Create a simple test image
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(test_image, "TEST IMAGE", (200, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)

            # Encode and send
            _, buffer = cv2.imencode('.jpg', test_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            files = {'file': ('test.jpg', buffer.tobytes(), 'image/jpeg')}

            start_time = time.time()
            response = requests.post(f"{self.server_url}/autonomous_drive", files=files, timeout=10)
            response_time = time.time() - start_time

            self.stats['total_requests'] += 1

            if response.status_code == 200:
                result = response.json()
                self.stats['successful_analyses'] += 1
                self.stats['avg_response_time'] = (
                    (self.stats['avg_response_time'] * (self.stats['successful_analyses'] - 1) + response_time) /
                    self.stats['successful_analyses']
                )
                self.last_analysis = result
                print(f"✅ Server test successful: {response_time:.2f}s")
                return True
            else:
                self.stats['failed_analyses'] += 1
                print(f"❌ Server test failed: {response.status_code}")
                return False

        except Exception as e:
            self.stats['failed_analyses'] += 1
            print(f"❌ Server connection failed: {e}")
            return False

    def generate_status_display(self):
        """Generate status display for dashboard"""
        if not self.last_analysis:
            return "Waiting for analysis..."

        # Extract key information
        steering = self.last_analysis.get('steering_angle', 0)
        speed = self.last_analysis.get('target_speed', 0)
        state = self.last_analysis.get('driving_state', 'unknown')
        confidence = self.last_analysis.get('lane_confidence', 0)

        return f"State: {state.upper()} | Steering: {steering:.2f} | Speed: {speed:.2f} | Confidence: {confidence:.2f}"

    def run_dashboard(self):
        """Run the dashboard server"""
        print("🌐 Dashboard starting at http://localhost:4000")
        self.app.run(host='0.0.0.0', port=4000, debug=False, threaded=True)

    def run(self):
        """Main run method"""
        # Test server connection first
        print("Testing server connection...")
        self.test_server_connection()

        # Start dashboard
        dashboard_thread = threading.Thread(target=self.run_dashboard, daemon=True)
        dashboard_thread.start()

        # Keep running and periodically test
        try:
            while True:
                time.sleep(5)  # Test every 5 seconds
                if self.auto_mode:
                    self.test_server_connection()
        except KeyboardInterrupt:
            print("\n👋 Dashboard shutting down...")

# Dashboard HTML Template
DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Autonomous Driving Dashboard</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #1e1e1e; color: white; }
        .container { max-width: 1200px; margin: 0 auto; }
        .header { text-align: center; margin-bottom: 30px; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 20px; border-radius: 10px; }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }
        .card { background: #2d2d2d; border-radius: 10px; padding: 20px; }
        .controls { display: flex; gap: 10px; margin: 20px 0; justify-content: center; }
        .btn { background: #4CAF50; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; font-size: 16px; }
        .btn:hover { background: #45a049; }
        .btn.stop { background: #f44336; }
        .btn.stop:hover { background: #da190b; }
        .btn.active { background: #2196F3; }
        .stats-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 15px; margin-bottom: 20px; }
        .stat-item { background: #3d3d3d; padding: 15px; border-radius: 8px; text-align: center; }
        .stat-number { font-size: 24px; font-weight: bold; color: #4CAF50; }
        .stat-label { font-size: 14px; color: #ccc; margin-top: 5px; }
        .analysis-display { font-size: 24px; font-weight: bold; text-align: center; padding: 20px; border-radius: 10px; margin-bottom: 20px; }
        .analysis-lane_following { background: #4CAF50; color: white; }
        .analysis-stopping { background: #F44336; color: white; }
        .analysis-emergency_stop_duckie { background: #FF5722; color: white; }
        .analysis-avoiding_obstacle { background: #FF9800; color: white; }
        .reasoning { background: #3d3d3d; padding: 15px; border-radius: 8px; font-family: monospace; }
        .status-indicator { display: inline-block; width: 12px; height: 12px; border-radius: 50%; margin-right: 8px; }
        .status-on { background: #4CAF50; }
        .status-off { background: #f44336; }
        .emergency-banner { background: #FF5722; color: white; padding: 10px; border-radius: 5px; text-align: center; margin-bottom: 20px; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚗 DuckieBot Autonomous Driving Dashboard</h1>
            <p>Real-time monitoring of autonomous navigation with lane following and duckling avoidance</p>
        </div>

        <div id="emergency-banner" class="emergency-banner" style="display: none;">
            🚨 EMERGENCY STOP ACTIVE 🚨
        </div>

        <div id="duck-emergency-banner" class="emergency-banner" style="display: none; background: #FF9800;">
            🦆 DUCK DETECTED - EMERGENCY STOP 🚨
        </div>

        <div class="controls">
            <button id="toggle-auto" class="btn" onclick="toggleAuto()">Auto Mode: ON</button>
            <button class="btn stop" onclick="emergencyStop()">🚨 Emergency Stop</button>
            <button class="btn" onclick="resumeDriving()">▶️ Resume</button>
            <button class="btn" onclick="testAnalysis()">🔍 Test Analysis</button>
        </div>

        <div class="grid">
            <div class="card">
                <h3>🧠 Current Analysis</h3>
                <div id="analysis-display" class="analysis-display">
                    Waiting for analysis...
                </div>
                <div id="reasoning" class="reasoning">
                    AI reasoning will appear here...
                </div>
                <div id="analysis-details" style="text-align: center; margin-top: 10px; color: #ccc;">
                    Details will appear here...
                </div>
            </div>

            <div class="card">
                <h3>📊 System Status</h3>
                <div id="status-display" class="reasoning">
                    Status information will appear here...
                </div>
                <div style="margin-top: 15px;">
                    <p><span id="auto-status" class="status-indicator status-on"></span>Auto Mode: <span id="auto-text">ON</span></p>
                    <p><span id="emergency-status" class="status-indicator status-off"></span>Emergency Stop: <span id="emergency-text">OFF</span></p>
                    <p><span id="duck-emergency-status" class="status-indicator status-off"></span>Duck Emergency Stop: <span id="duck-emergency-text">OFF</span></p>
                </div>
            </div>
        </div>

        <div class="card">
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
                    <div id="emergency-count" class="stat-number">0</div>
                    <div class="stat-label">Emergency Stops</div>
                </div>
                <div class="stat-item">
                    <div id="duck-emergency-count" class="stat-number">0</div>
                    <div class="stat-label">Duck Emergency Stops</div>
                </div>
                <div class="stat-item">
                    <div id="duck-detection-count" class="stat-number">0</div>
                    <div class="stat-label">Duck Detections</div>
                </div>
                <div class="stat-item">
                    <div id="lane-confidence" class="stat-number">0.00</div>
                    <div class="stat-label">Lane Confidence</div>
                </div>
                <div class="stat-item">
                    <div id="current-state" class="stat-number">unknown</div>
                    <div class="stat-label">Current State</div>
                </div>
            </div>
        </div>

        <div class="card">
            <h3>🎯 Lane Following & Safety Info</h3>
            <div id="lane-info" class="reasoning">
                Lane following and safety information will appear here...
            </div>
        </div>
    </div>

    <script>
        let autoMode = true;
        let emergencyOverride = false;

        function updateData() {
            // Update latest analysis
            fetch('/api/latest')
                .then(response => response.json())
                .then(data => {
                    if (data.analysis) {
                        const analysis = data.analysis;
                        const state = analysis.driving_state || 'unknown';

                        // Update analysis display
                        const analysisEl = document.getElementById('analysis-display');
                        analysisEl.textContent = state.toUpperCase().replace('_', ' ');
                        analysisEl.className = 'analysis-display analysis-' + state;

                        // Update reasoning
                        document.getElementById('reasoning').textContent =
                            analysis.debug_info || analysis.reasoning || 'No reasoning available';

                        // Update details
                        const steering = analysis.steering_angle || 0;
                        const speed = analysis.target_speed || 0;
                        const confidence = analysis.lane_confidence || 0;
                        document.getElementById('analysis-details').textContent =
                            `Steering: ${steering.toFixed(2)} | Speed: ${speed.toFixed(2)} | Confidence: ${confidence.toFixed(2)}`;

                        // Update status
                        document.getElementById('status-display').textContent =
                            `State: ${state} | Confidence: ${confidence.toFixed(2)} | Emergency: ${emergencyOverride ? 'ACTIVE' : 'CLEAR'}`;

                        // Update lane info
                        const laneInfo = analysis.lane_info || {};
                        document.getElementById('lane-info').textContent =
                            `Lane Offset: ${(laneInfo.lane_center_offset || 0).toFixed(2)} | ` +
                            `Lane Confidence: ${(laneInfo.confidence || 0).toFixed(2)} | ` +
                            `Obstacle Detected: ${analysis.obstacle_detected ? 'YES' : 'NO'}`;
                    }

                    // Update status indicators
                    autoMode = data.auto_mode;
                    emergencyOverride = data.emergency_override;
                    const duckEmergency = data.duck_emergency_stop;
                    const duckDetectionCount = data.duck_detection_count;
                    updateStatusIndicators(duckEmergency);

                    // Show/hide emergency banners
                    const banner = document.getElementById('emergency-banner');
                    banner.style.display = emergencyOverride ? 'block' : 'none';

                    const duckBanner = document.getElementById('duck-emergency-banner');
                    duckBanner.style.display = duckEmergency ? 'block' : 'none';
                });

            // Update stats
            fetch('/api/stats')
                .then(response => response.json())
                .then(stats => {
                    document.getElementById('total-requests').textContent = stats.total_requests;

                    const successRate = stats.total_requests > 0 ?
                        (100 * stats.successful_analyses / stats.total_requests).toFixed(1) : 0;
                    document.getElementById('success-rate').textContent = successRate + '%';

                    document.getElementById('avg-response').textContent = stats.avg_response_time.toFixed(2) + 's';
                    document.getElementById('emergency-count').textContent = stats.emergency_stops;
                    document.getElementById('duck-emergency-count').textContent = stats.duck_emergency_stops || 0;
                    document.getElementById('duck-detection-count').textContent = stats.duck_detection_events || 0;
                    document.getElementById('lane-confidence').textContent = stats.lane_following_confidence.toFixed(2);
                    document.getElementById('current-state').textContent = stats.current_state;
                });
        }

                function updateStatusIndicators(duckEmergency) {
            // Auto mode status
            const autoBtn = document.getElementById('toggle-auto');
            const autoStatus = document.getElementById('auto-status');
            const autoText = document.getElementById('auto-text');

            autoBtn.textContent = `Auto Mode: ${autoMode ? 'ON' : 'OFF'}`;
            autoBtn.className = autoMode ? 'btn active' : 'btn';
            autoStatus.className = autoMode ? 'status-indicator status-on' : 'status-indicator status-off';
            autoText.textContent = autoMode ? 'ON' : 'OFF';

            // Emergency stop status
            const emergencyStatus = document.getElementById('emergency-status');
            const emergencyText = document.getElementById('emergency-text');

            emergencyStatus.className = emergencyOverride ? 'status-indicator status-on' : 'status-indicator status-off';
            emergencyText.textContent = emergencyOverride ? 'ACTIVE' : 'OFF';

            // Duck emergency stop status
            const duckEmergencyStatus = document.getElementById('duck-emergency-status');
            const duckEmergencyText = document.getElementById('duck-emergency-text');

            duckEmergencyStatus.className = duckEmergency ? 'status-indicator status-on' : 'status-indicator status-off';
            duckEmergencyText.textContent = duckEmergency ? 'ACTIVE' : 'OFF';
        }

        function toggleAuto() {
            fetch('/api/toggle_auto')
                .then(response => response.json())
                .then(data => {
                    autoMode = data.auto_mode;
                    updateStatusIndicators();
                });
        }

        function emergencyStop() {
            fetch('/api/emergency_stop')
                .then(response => response.json())
                .then(data => {
                    emergencyOverride = true;
                    updateStatusIndicators();
                    document.getElementById('emergency-banner').style.display = 'block';
                });
        }

        function resumeDriving() {
            fetch('/api/resume')
                .then(response => response.json())
                .then(data => {
                    emergencyOverride = false;
                    updateStatusIndicators();
                    document.getElementById('emergency-banner').style.display = 'none';
                });
        }

        function testAnalysis() {
            fetch('/api/test_analysis')
                .then(response => response.json())
                .then(data => {
                    console.log('Analysis test triggered');
                });
        }

        // Update every 2 seconds
        setInterval(updateData, 2000);
        updateData(); // Initial load
    </script>
</body>
</html>
"""

def main():
    import argparse

    parser = argparse.ArgumentParser(description='Autonomous Driving Dashboard')
    parser.add_argument('--server-url', default='http://192.168.140.179:8000',
                       help='Autonomous driving server URL')

    args = parser.parse_args()

    dashboard = AutonomousDrivingDashboard(server_url=args.server_url)
    dashboard.run()

if __name__ == "__main__":
    main()
