#!/usr/bin/env python3
"""
Beautiful Frontend for llama.cpp AutoDuck VLM System
Real-time monitoring dashboard with advanced analytics and controls

Features:
- Live camera feed with AI overlays
- Real-time performance metrics and charts
- Decision distribution analytics  
- Response time monitoring
- System health indicators
- Interactive controls
"""

from flask import Flask, render_template, jsonify, Response, request
import requests
import json
import time
from datetime import datetime, timedelta
import threading
import queue
import cv2
import base64

app = Flask(__name__)

class LlamaCppFrontend:
    def __init__(self, vlm_server_url="http://localhost:5000"):
        self.vlm_server_url = vlm_server_url
        self.performance_history = []
        self.decision_history = []
        self.running = True
        
        # Start background monitoring
        self.monitor_thread = threading.Thread(target=self.monitor_performance, daemon=True)
        self.monitor_thread.start()
        
    def monitor_performance(self):
        """Background monitoring of VLM system performance"""
        while self.running:
            try:
                # Get latest stats from VLM system
                response = requests.get(f"{self.vlm_server_url}/api/stats", timeout=2)
                if response.status_code == 200:
                    stats = response.json()
                    timestamp = datetime.now()
                    
                    # Store performance history
                    self.performance_history.append({
                        'timestamp': timestamp.isoformat(),
                        'response_time': stats.get('avg_response_time', 0),
                        'fps': stats.get('current_fps', 0),
                        'success_rate': (stats.get('successful_requests', 0) / max(1, stats.get('total_requests', 1))) * 100
                    })
                    
                    # Keep only last 100 data points
                    if len(self.performance_history) > 100:
                        self.performance_history = self.performance_history[-100:]
                        
                    # Get latest decision
                    decision_response = requests.get(f"{self.vlm_server_url}/api/latest", timeout=2)
                    if decision_response.status_code == 200:
                        decision_data = decision_response.json()
                        if decision_data.get('result'):
                            self.decision_history.append({
                                'timestamp': timestamp.isoformat(),
                                'decision': decision_data['result'].get('decision', 'UNKNOWN'),
                                'response_time': decision_data['result'].get('response_time', 0),
                                'reasoning': decision_data['result'].get('reasoning', '')
                            })
                            
                            # Keep only last 50 decisions
                            if len(self.decision_history) > 50:
                                self.decision_history = self.decision_history[-50:]
                                
            except Exception as e:
                print(f"Monitor error: {e}")
                
            time.sleep(1)  # Update every second

frontend = LlamaCppFrontend()

@app.route('/')
def dashboard():
    """Main dashboard page"""
    return render_template_string(DASHBOARD_HTML)

@app.route('/api/performance_history')
def get_performance_history():
    """Get performance history for charts"""
    return jsonify(frontend.performance_history)

@app.route('/api/decision_history')
def get_decision_history():
    """Get recent decision history"""
    return jsonify(frontend.decision_history)

@app.route('/api/system_status')
def get_system_status():
    """Check if VLM system is running"""
    try:
        response = requests.get(f"{frontend.vlm_server_url}/api/stats", timeout=2)
        if response.status_code == 200:
            return jsonify({'status': 'online', 'timestamp': datetime.now().isoformat()})
    except:
        pass
    return jsonify({'status': 'offline', 'timestamp': datetime.now().isoformat()})

@app.route('/api/current_stats')
def get_current_stats():
    """Proxy current stats from VLM system"""
    try:
        response = requests.get(f"{frontend.vlm_server_url}/api/stats", timeout=2)
        if response.status_code == 200:
            return response.text, 200, {'Content-Type': 'application/json'}
    except:
        pass
    return jsonify({'error': 'VLM system unavailable'}), 503

@app.route('/api/latest_analysis')
def get_latest_analysis():
    """Proxy latest analysis from VLM system"""
    try:
        response = requests.get(f"{frontend.vlm_server_url}/api/latest", timeout=2)
        if response.status_code == 200:
            return response.text, 200, {'Content-Type': 'application/json'}
    except:
        pass
    return jsonify({'error': 'VLM system unavailable'}), 503

@app.route('/video_feed')
def video_feed():
    """Proxy video feed from VLM system"""
    try:
        def generate():
            response = requests.get(f"{frontend.vlm_server_url}/video_feed", stream=True, timeout=10)
            for chunk in response.iter_content(chunk_size=1024):
                yield chunk
        return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
    except:
        # Return placeholder image if VLM system unavailable
        return Response(b'', mimetype='multipart/x-mixed-replace; boundary=frame')

# Beautiful HTML Template
DASHBOARD_HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🚀 AutoDuck VLM - llama.cpp Ultra-Fast Dashboard</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #0f0f23 0%, #1a1a2e 50%, #16213e 100%);
            color: #ffffff;
            min-height: 100vh;
            overflow-x: hidden;
        }
        
        .header {
            background: linear-gradient(90deg, #ff6b35 0%, #f7931e 50%, #ff6b35 100%);
            padding: 20px 0;
            text-align: center;
            box-shadow: 0 4px 20px rgba(255, 107, 53, 0.3);
            margin-bottom: 30px;
        }
        
        .header h1 {
            font-size: 2.5rem;
            font-weight: 700;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
            margin-bottom: 10px;
        }
        
        .header p {
            font-size: 1.1rem;
            opacity: 0.9;
        }
        
        .status-bar {
            display: flex;
            justify-content: center;
            gap: 30px;
            margin-bottom: 30px;
            flex-wrap: wrap;
        }
        
        .status-item {
            background: rgba(255, 255, 255, 0.1);
            backdrop-filter: blur(10px);
            padding: 15px 25px;
            border-radius: 15px;
            border: 1px solid rgba(255, 255, 255, 0.2);
            display: flex;
            align-items: center;
            gap: 10px;
            transition: all 0.3s ease;
        }
        
        .status-item:hover {
            transform: translateY(-3px);
            box-shadow: 0 10px 25px rgba(255, 107, 53, 0.3);
        }
        
        .status-online { border-left: 4px solid #4CAF50; }
        .status-offline { border-left: 4px solid #f44336; }
        
        .container {
            max-width: 1400px;
            margin: 0 auto;
            padding: 0 20px;
        }
        
        .grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 25px;
            margin-bottom: 30px;
        }
        
        .grid-3 {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 25px;
            margin-bottom: 30px;
        }
        
        .card {
            background: rgba(255, 255, 255, 0.05);
            backdrop-filter: blur(15px);
            border-radius: 20px;
            padding: 25px;
            border: 1px solid rgba(255, 255, 255, 0.1);
            transition: all 0.3s ease;
            position: relative;
            overflow: hidden;
        }
        
        .card::before {
            content: '';
            position: absolute;
            top: 0;
            left: 0;
            right: 0;
            height: 3px;
            background: linear-gradient(90deg, #ff6b35, #f7931e, #ff6b35);
            opacity: 0;
            transition: opacity 0.3s ease;
        }
        
        .card:hover::before {
            opacity: 1;
        }
        
        .card:hover {
            transform: translateY(-5px);
            box-shadow: 0 15px 35px rgba(255, 107, 53, 0.2);
            border-color: rgba(255, 107, 53, 0.3);
        }
        
        .card h3 {
            font-size: 1.3rem;
            margin-bottom: 20px;
            color: #ff6b35;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        
        .video-container {
            position: relative;
            border-radius: 15px;
            overflow: hidden;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
        }
        
        .video-feed {
            width: 100%;
            height: auto;
            display: block;
        }
        
        .decision-display {
            text-align: center;
            padding: 30px;
            border-radius: 15px;
            margin-bottom: 20px;
            font-size: 3rem;
            font-weight: bold;
            text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
            transition: all 0.3s ease;
        }
        
        .decision-FORWARD {
            background: linear-gradient(135deg, #4CAF50, #8BC34A);
            box-shadow: 0 10px 30px rgba(76, 175, 80, 0.3);
        }
        
        .decision-LEFT {
            background: linear-gradient(135deg, #FF9800, #FFC107);
            box-shadow: 0 10px 30px rgba(255, 152, 0, 0.3);
        }
        
        .decision-RIGHT {
            background: linear-gradient(135deg, #9C27B0, #E91E63);
            box-shadow: 0 10px 30px rgba(156, 39, 176, 0.3);
        }
        
        .decision-STOP {
            background: linear-gradient(135deg, #F44336, #E91E63);
            box-shadow: 0 10px 30px rgba(244, 67, 54, 0.3);
        }
        
        .reasoning {
            background: rgba(0, 0, 0, 0.3);
            padding: 20px;
            border-radius: 10px;
            font-family: 'Courier New', monospace;
            line-height: 1.6;
            border-left: 4px solid #ff6b35;
        }
        
        .stats-grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin-bottom: 25px;
        }
        
        .stat-item {
            background: rgba(255, 255, 255, 0.08);
            padding: 20px;
            border-radius: 15px;
            text-align: center;
            transition: all 0.3s ease;
            border: 1px solid rgba(255, 255, 255, 0.1);
        }
        
        .stat-item:hover {
            transform: scale(1.05);
            background: rgba(255, 107, 53, 0.1);
        }
        
        .stat-number {
            font-size: 2.2rem;
            font-weight: bold;
            background: linear-gradient(45deg, #ff6b35, #f7931e);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
            margin-bottom: 8px;
        }
        
        .stat-label {
            font-size: 0.9rem;
            color: #cccccc;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        
        .chart-container {
            position: relative;
            height: 300px;
            margin-top: 20px;
        }
        
        .decision-history {
            max-height: 400px;
            overflow-y: auto;
            margin-top: 20px;
        }
        
        .decision-item {
            background: rgba(255, 255, 255, 0.05);
            padding: 15px;
            margin-bottom: 10px;
            border-radius: 10px;
            border-left: 4px solid;
            transition: all 0.3s ease;
        }
        
        .decision-item:hover {
            background: rgba(255, 255, 255, 0.1);
            transform: translateX(5px);
        }
        
        .decision-item.FORWARD { border-left-color: #4CAF50; }
        .decision-item.LEFT { border-left-color: #FF9800; }
        .decision-item.RIGHT { border-left-color: #9C27B0; }
        .decision-item.STOP { border-left-color: #F44336; }
        
        .decision-header {
            display: flex;
            justify-content: between;
            align-items: center;
            margin-bottom: 8px;
        }
        
        .decision-time {
            font-size: 0.8rem;
            color: #999;
        }
        
        .decision-reasoning {
            font-size: 0.9rem;
            line-height: 1.4;
            color: #ddd;
        }
        
        .loading {
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
        
        .performance-indicator {
            display: flex;
            align-items: center;
            gap: 10px;
            margin-bottom: 15px;
        }
        
        .indicator-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            animation: pulse-dot 2s infinite;
        }
        
        .indicator-dot.excellent { background: #4CAF50; }
        .indicator-dot.good { background: #8BC34A; }
        .indicator-dot.average { background: #FFC107; }
        .indicator-dot.poor { background: #FF9800; }
        .indicator-dot.critical { background: #F44336; }
        
        @keyframes pulse-dot {
            0%, 100% { transform: scale(1); }
            50% { transform: scale(1.2); }
        }
        
        @media (max-width: 768px) {
            .grid, .grid-3 {
                grid-template-columns: 1fr;
            }
            
            .header h1 {
                font-size: 1.8rem;
            }
            
            .decision-display {
                font-size: 2rem;
                padding: 20px;
            }
            
            .status-bar {
                flex-direction: column;
                align-items: center;
                gap: 15px;
            }
        }
        
        .fullscreen-btn {
            position: absolute;
            top: 10px;
            right: 10px;
            background: rgba(0,0,0,0.7);
            color: white;
            border: none;
            padding: 8px 12px;
            border-radius: 5px;
            cursor: pointer;
            font-size: 0.8rem;
        }
        
        .fullscreen-btn:hover {
            background: rgba(255, 107, 53, 0.8);
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🚀 AutoDuck VLM Dashboard</h1>
        <p>Ultra-Fast Autonomous Driving with llama.cpp + CUDA Acceleration</p>
    </div>
    
    <div class="container">
        <div class="status-bar">
            <div id="system-status" class="status-item status-offline">
                <span>🔴</span>
                <span>System: Checking...</span>
            </div>
            <div id="performance-indicator" class="status-item">
                <div class="indicator-dot excellent"></div>
                <span>Performance: Excellent</span>
            </div>
            <div id="llama-server-status" class="status-item">
                <span>🐋</span>
                <span>llama.cpp Server: Active</span>
            </div>
        </div>
        
        <div class="grid">
            <div class="card">
                <h3>📹 Live Camera Feed</h3>
                <div class="video-container">
                    <img id="video-feed" class="video-feed" src="/video_feed" alt="Camera Feed">
                    <button class="fullscreen-btn" onclick="toggleFullscreen()">⛶ Fullscreen</button>
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
                <div id="analysis-time" style="text-align: center; margin-top: 15px; color: #ccc;">
                    Response time: --
                </div>
            </div>
        </div>
        
        <div class="grid-3">
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
                        <div class="stat-label">Avg Response</div>
                    </div>
                    <div class="stat-item">
                        <div id="current-fps" class="stat-number">0.00</div>
                        <div class="stat-label">Current FPS</div>
                    </div>
                </div>
            </div>
            
            <div class="card">
                <h3>🎮 Decision Distribution</h3>
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
                <h3>📈 System Health</h3>
                <div class="performance-indicator">
                    <div id="response-indicator" class="indicator-dot excellent"></div>
                    <span>Response Time: <span id="response-health">Excellent</span></span>
                </div>
                <div class="performance-indicator">
                    <div id="fps-indicator" class="indicator-dot excellent"></div>
                    <span>FPS Performance: <span id="fps-health">Excellent</span></span>
                </div>
                <div class="performance-indicator">
                    <div id="reliability-indicator" class="indicator-dot excellent"></div>
                    <span>Reliability: <span id="reliability-health">Excellent</span></span>
                </div>
            </div>
        </div>
        
        <div class="grid">
            <div class="card">
                <h3>📈 Performance History</h3>
                <div class="chart-container">
                    <canvas id="performance-chart"></canvas>
                </div>
            </div>
            
            <div class="card">
                <h3>📋 Recent Decisions</h3>
                <div id="decision-history" class="decision-history">
                    Loading decision history...
                </div>
            </div>
        </div>
    </div>

    <script>
        let performanceChart;
        
        // Initialize performance chart
        function initPerformanceChart() {
            const ctx = document.getElementById('performance-chart').getContext('2d');
            performanceChart = new Chart(ctx, {
                type: 'line',
                data: {
                    datasets: [
                        {
                            label: 'Response Time (s)',
                            data: [],
                            borderColor: '#ff6b35',
                            backgroundColor: 'rgba(255, 107, 53, 0.1)',
                            tension: 0.4
                        },
                        {
                            label: 'FPS',
                            data: [],
                            borderColor: '#4CAF50',
                            backgroundColor: 'rgba(76, 175, 80, 0.1)',
                            tension: 0.4
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    plugins: {
                        legend: {
                            labels: { color: '#ffffff' }
                        }
                    },
                    scales: {
                        x: {
                            ticks: { color: '#cccccc' },
                            grid: { color: 'rgba(255, 255, 255, 0.1)' }
                        },
                        y: {
                            ticks: { color: '#cccccc' },
                            grid: { color: 'rgba(255, 255, 255, 0.1)' }
                        }
                    }
                }
            });
        }
        
        // Update system status
        function updateSystemStatus() {
            fetch('/api/system_status')
                .then(response => response.json())
                .then(data => {
                    const statusEl = document.getElementById('system-status');
                    if (data.status === 'online') {
                        statusEl.className = 'status-item status-online';
                        statusEl.innerHTML = '<span>🟢</span><span>System: Online</span>';
                    } else {
                        statusEl.className = 'status-item status-offline';
                        statusEl.innerHTML = '<span>🔴</span><span>System: Offline</span>';
                    }
                })
                .catch(() => {
                    const statusEl = document.getElementById('system-status');
                    statusEl.className = 'status-item status-offline';
                    statusEl.innerHTML = '<span>🔴</span><span>System: Error</span>';
                });
        }
        
        // Update current stats
        function updateStats() {
            fetch('/api/current_stats')
                .then(response => response.json())
                .then(stats => {
                    document.getElementById('total-requests').textContent = stats.total_requests || 0;
                    
                    const successRate = stats.total_requests > 0 ? 
                        (100 * stats.successful_requests / stats.total_requests).toFixed(1) : 0;
                    document.getElementById('success-rate').textContent = successRate + '%';
                    
                    document.getElementById('avg-response').textContent = (stats.avg_response_time || 0).toFixed(2) + 's';
                    document.getElementById('current-fps').textContent = (stats.current_fps || 0).toFixed(2);
                    
                    // Decision counts
                    const decisions = stats.decisions || {};
                    document.getElementById('forward-count').textContent = decisions.FORWARD || 0;
                    document.getElementById('left-count').textContent = decisions.LEFT || 0;
                    document.getElementById('right-count').textContent = decisions.RIGHT || 0;
                    document.getElementById('stop-count').textContent = decisions.STOP || 0;
                    
                    // Update health indicators
                    updateHealthIndicators(stats);
                })
                .catch(console.error);
        }
        
        // Update latest analysis
        function updateLatestAnalysis() {
            fetch('/api/latest_analysis')
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
                })
                .catch(console.error);
        }
        
        // Update performance chart
        function updatePerformanceChart() {
            fetch('/api/performance_history')
                .then(response => response.json())
                .then(history => {
                    const labels = history.map((_, i) => i);
                    const responseData = history.map(h => h.response_time);
                    const fpsData = history.map(h => h.fps);
                    
                    performanceChart.data.labels = labels;
                    performanceChart.data.datasets[0].data = responseData;
                    performanceChart.data.datasets[1].data = fpsData;
                    performanceChart.update('none');
                })
                .catch(console.error);
        }
        
        // Update decision history
        function updateDecisionHistory() {
            fetch('/api/decision_history')
                .then(response => response.json())
                .then(decisions => {
                    const historyEl = document.getElementById('decision-history');
                    historyEl.innerHTML = decisions.slice(-10).reverse().map(d => `
                        <div class="decision-item ${d.decision}">
                            <div class="decision-header">
                                <strong>${d.decision}</strong>
                                <span class="decision-time">${new Date(d.timestamp).toLocaleTimeString()}</span>
                            </div>
                            <div class="decision-reasoning">${d.reasoning}</div>
                            <div style="font-size: 0.8rem; color: #999; margin-top: 5px;">
                                Response: ${d.response_time.toFixed(2)}s
                            </div>
                        </div>
                    `).join('');
                })
                .catch(console.error);
        }
        
        // Update health indicators
        function updateHealthIndicators(stats) {
            const responseTime = stats.avg_response_time || 0;
            const fps = stats.current_fps || 0;
            const successRate = stats.total_requests > 0 ? 
                (stats.successful_requests / stats.total_requests) * 100 : 0;
            
            // Response time health
            const responseIndicator = document.getElementById('response-indicator');
            const responseHealth = document.getElementById('response-health');
            if (responseTime < 1.0) {
                responseIndicator.className = 'indicator-dot excellent';
                responseHealth.textContent = 'Excellent';
            } else if (responseTime < 2.0) {
                responseIndicator.className = 'indicator-dot good';
                responseHealth.textContent = 'Good';
            } else if (responseTime < 3.0) {
                responseIndicator.className = 'indicator-dot average';
                responseHealth.textContent = 'Average';
            } else {
                responseIndicator.className = 'indicator-dot poor';
                responseHealth.textContent = 'Poor';
            }
            
            // FPS health
            const fpsIndicator = document.getElementById('fps-indicator');
            const fpsHealth = document.getElementById('fps-health');
            if (fps > 0.8) {
                fpsIndicator.className = 'indicator-dot excellent';
                fpsHealth.textContent = 'Excellent';
            } else if (fps > 0.5) {
                fpsIndicator.className = 'indicator-dot good';
                fpsHealth.textContent = 'Good';
            } else if (fps > 0.3) {
                fpsIndicator.className = 'indicator-dot average';
                fpsHealth.textContent = 'Average';
            } else {
                fpsIndicator.className = 'indicator-dot poor';
                fpsHealth.textContent = 'Poor';
            }
            
            // Reliability health
            const reliabilityIndicator = document.getElementById('reliability-indicator');
            const reliabilityHealth = document.getElementById('reliability-health');
            if (successRate >= 95) {
                reliabilityIndicator.className = 'indicator-dot excellent';
                reliabilityHealth.textContent = 'Excellent';
            } else if (successRate >= 90) {
                reliabilityIndicator.className = 'indicator-dot good';
                reliabilityHealth.textContent = 'Good';
            } else if (successRate >= 80) {
                reliabilityIndicator.className = 'indicator-dot average';
                reliabilityHealth.textContent = 'Average';
            } else {
                reliabilityIndicator.className = 'indicator-dot poor';
                reliabilityHealth.textContent = 'Poor';
            }
        }
        
        // Toggle fullscreen for video
        function toggleFullscreen() {
            const video = document.getElementById('video-feed');
            if (video.requestFullscreen) {
                video.requestFullscreen();
            } else if (video.webkitRequestFullscreen) {
                video.webkitRequestFullscreen();
            } else if (video.msRequestFullscreen) {
                video.msRequestFullscreen();
            }
        }
        
        // Initialize everything
        function init() {
            initPerformanceChart();
            updateSystemStatus();
            updateStats();
            updateLatestAnalysis();
            updateDecisionHistory();
            
            // Set up intervals
            setInterval(updateSystemStatus, 5000);
            setInterval(updateStats, 1000);
            setInterval(updateLatestAnalysis, 1000);
            setInterval(updatePerformanceChart, 2000);
            setInterval(updateDecisionHistory, 3000);
        }
        
        // Start when page loads
        document.addEventListener('DOMContentLoaded', init);
    </script>
</body>
</html>
"""

if __name__ == '__main__':
    print("🚀 Starting llama.cpp AutoDuck VLM Frontend")
    print("📊 Dashboard will be available at: http://localhost:3000")
    print("🔗 Make sure llama.cpp VLM system is running at: http://localhost:5000")
    
    app.run(host='0.0.0.0', port=3000, debug=False, threaded=True) 