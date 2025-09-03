#!/usr/bin/env python3
"""
Quick starter for DuckieBot VLM Navigation System
Handles llama.cpp server startup and navigation system launch
"""

import subprocess
import time
import requests
import sys
import os
import argparse
import signal

def check_docker():
    """Check if Docker is available"""
    try:
        subprocess.run(["docker", "--version"], check=True, capture_output=True)
        return True
    except:
        print("❌ Docker not found. Please install Docker first.")
        return False

def check_llama_server(url="http://localhost:8080"):
    """Check if llama.cpp server is running"""
    try:
        response = requests.get(f"{url}/health", timeout=3)
        return response.status_code == 200
    except:
        return False

def start_llama_server():
    """Start llama.cpp server if not running"""
    if check_llama_server():
        print("✅ llama.cpp server already running")
        return True
    
    print("🚀 Starting llama.cpp server with Qwen2.5-VL...")
    
    # Remove existing container if any
    subprocess.run(["docker", "stop", "llama-vlm-server"], capture_output=True)
    subprocess.run(["docker", "rm", "llama-vlm-server"], capture_output=True)
    
    # Start server
    cmd = [
        "docker", "run", "--gpus", "all", "-d", 
        "-p", "8080:8080", "--name", "llama-vlm-server",
        "ghcr.io/ggml-org/llama.cpp:server-cuda",
        "-hf", "ggml-org/Qwen2.5-VL-7B-Instruct-GGUF",
        "--host", "0.0.0.0", "--port", "8080", "--n-gpu-layers", "99"
    ]
    
    try:
        subprocess.run(cmd, check=True)
        print("⏳ Waiting for server to start...")
        
        # Wait for server to be ready
        for i in range(60):  # Wait up to 2 minutes
            if check_llama_server():
                print("✅ llama.cpp server started successfully")
                return True
            time.sleep(2)
            print(".", end="", flush=True)
        
        print("\n❌ Server failed to start in 2 minutes")
        return False
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to start server: {e}")
        return False

def check_ros():
    """Check if ROS is available"""
    try:
        subprocess.run(["roscore", "--help"], check=True, capture_output=True)
        return True
    except:
        print("❌ ROS not found. Make sure ROS is installed and sourced.")
        return False

def check_duckiebot_connection(robot_name):
    """Check if DuckieBot is accessible"""
    try:
        # Try to ping the robot
        result = subprocess.run(["ping", "-c", "1", f"{robot_name}.local"], 
                              capture_output=True, timeout=5)
        if result.returncode == 0:
            print(f"✅ DuckieBot {robot_name} is reachable")
            return True
        else:
            print(f"⚠️ Could not ping {robot_name}.local")
            return False
    except:
        print(f"⚠️ Could not check connection to {robot_name}")
        return False

def start_navigation_system(robot_name, llama_url="http://localhost:8080", fps=2.0):
    """Start the VLM navigation system"""
    print(f"🤖 Starting VLM Navigation for {robot_name}...")
    
    # Check if we're in a ROS environment
    if 'ROS_MASTER_URI' not in os.environ:
        print("⚠️ ROS environment not detected. Make sure to source your ROS setup.")
    
    try:
        # Start the navigation system
        cmd = [
            sys.executable, 
            "duckiebot_vlm_navigation.py",
            robot_name,
            "--llama-url", llama_url,
            "--fps", str(fps)
        ]
        
        print(f"📋 Running: {' '.join(cmd)}")
        subprocess.run(cmd)
        
    except KeyboardInterrupt:
        print("\n✅ Navigation system stopped by user")
    except Exception as e:
        print(f"❌ Error starting navigation: {e}")

def main():
    parser = argparse.ArgumentParser(description='Start DuckieBot VLM Navigation System')
    parser.add_argument('robot_name', help='Name of the DuckieBot (e.g., ducky)')
    parser.add_argument('--llama-url', default='http://localhost:8080',
                       help='URL of llama.cpp server (default: http://localhost:8080)')
    parser.add_argument('--fps', type=float, default=2.0,
                       help='VLM processing FPS (default: 2.0)')
    parser.add_argument('--skip-server-start', action='store_true',
                       help='Skip starting llama.cpp server')
    parser.add_argument('--skip-connection-check', action='store_true',
                       help='Skip DuckieBot connection check')
    
    args = parser.parse_args()
    
    print("🚀 DuckieBot VLM Navigation Startup")
    print("="*50)
    
    # Check prerequisites
    if not check_docker():
        return 1
    
    if not check_ros():
        return 1
    
    # Check DuckieBot connection
    if not args.skip_connection_check:
        if not check_duckiebot_connection(args.robot_name):
            if input("Continue anyway? (y/n): ").lower() != 'y':
                return 1
    
    # Start llama.cpp server
    if not args.skip_server_start:
        if not start_llama_server():
            return 1
    else:
        if not check_llama_server(args.llama_url):
            print(f"❌ llama.cpp server not running at {args.llama_url}")
            return 1
        print("✅ llama.cpp server is running")
    
    print("\n📋 System Ready!")
    print(f"🤖 Robot: {args.robot_name}")
    print(f"🧠 VLM Server: {args.llama_url}")
    print(f"⚡ Processing FPS: {args.fps}")
    print("\n🚨 Emergency Controls:")
    print("  Ctrl+C: Stop navigation system")
    print("  rostopic pub /vlm_navigator/emergency_stop std_msgs/Bool 'data: true'")
    print("\n🔧 Debugging:")
    print("  rostopic echo /vlm_navigator/current_decision")
    print("  rostopic echo /vlm_navigator/performance")
    print("  rosrun rqt_image_view rqt_image_view (to see camera feed)")
    
    # Ask if user wants to start navigation
    if input("\nStart VLM navigation now? (y/n): ").lower() == 'y':
        start_navigation_system(args.robot_name, args.llama_url, args.fps)
    
    print("\n✅ Startup complete!")
    return 0

if __name__ == "__main__":
    sys.exit(main())
