#!/usr/bin/env python3
"""
Integrated VLM Keyboard Control Launcher
Launches all components needed for VLM-based Duckiebot control

Components:
1. llama.cpp server with Qwen2.5-VL
2. VLM Keyboard Control Bridge
3. Duckiebot video streamer
4. Optional keyboard GUI launcher
"""

import subprocess
import time
import os
import sys
import signal
import argparse
import requests
from threading import Thread

class VLMControlLauncher:
    def __init__(self, robot_name, laptop_ip=None):
        self.robot_name = robot_name
        self.laptop_ip = laptop_ip or self.get_local_ip()
        self.processes = []
        
    def get_local_ip(self):
        """Get local IP address"""
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
        except Exception:
            ip = '127.0.0.1'
        finally:
            s.close()
        return ip
    
    def check_llama_server(self):
        """Check if llama.cpp server is running"""
        try:
            response = requests.get("http://localhost:8080/health", timeout=2)
            return response.status_code == 200
        except:
            return False
    
    def start_llama_server(self):
        """Start llama.cpp server if not running"""
        if self.check_llama_server():
            print("✅ llama.cpp server already running")
            return True
        
        print("🚀 Starting llama.cpp server with Qwen2.5-VL...")
        cmd = [
            "docker", "run", "--gpus", "all", "-p", "8080:8080",
            "--name", "llama-vlm-server",
            "ghcr.io/ggml-org/llama.cpp:server-cuda",
            "-hf", "ggml-org/Qwen2.5-VL-7B-Instruct-GGUF",
            "--host", "0.0.0.0", "--port", "8080",
            "--n-gpu-layers", "99", "--ctx-size", "1024",
            "--batch-size", "256", "--threads", "4", "--cont-batching"
        ]
        
        process = subprocess.Popen(cmd)
        self.processes.append(process)
        
        # Wait for server to start
        print("⏳ Waiting for llama.cpp server to start...")
        for i in range(30):
            if self.check_llama_server():
                print("✅ llama.cpp server started successfully")
                return True
            time.sleep(2)
        
        print("❌ Failed to start llama.cpp server")
        return False
    
    def start_keyboard_bridge(self):
        """Start VLM Keyboard Control Bridge"""
        print("🌉 Starting VLM Keyboard Control Bridge...")
        cmd = [sys.executable, "keyboard_control_bridge.py"]
        process = subprocess.Popen(cmd, cwd="vlm_server")
        self.processes.append(process)
        
        # Wait for bridge to start
        time.sleep(3)
        try:
            response = requests.get("http://localhost:8000/control/status", timeout=2)
            if response.status_code == 200:
                print("✅ Keyboard Control Bridge started")
                print("📱 Dashboard available at: http://localhost:8000")
                return True
        except:
            pass
        
        print("⚠️  Keyboard Control Bridge may not be ready")
        return False
    
    def start_video_streamer(self):
        """Start Duckiebot video streamer"""
        print(f"📹 Starting video streamer for {self.robot_name}...")
        cmd = [
            sys.executable, "duckiebot_vlm_streamer.py",
            self.robot_name,
            "--bridge-url", "http://localhost:8000"
        ]
        
        # Set ROS_MASTER_URI for remote robot
        env = os.environ.copy()
        env['ROS_MASTER_URI'] = f"http://{self.robot_name}.local:11311"
        env['ROS_IP'] = self.laptop_ip
        
        process = subprocess.Popen(cmd, cwd="vlm_server", env=env)
        self.processes.append(process)
        print(f"✅ Video streamer started (ROS_MASTER_URI={env['ROS_MASTER_URI']})")
    
    def launch_keyboard_gui(self):
        """Launch Duckietown keyboard control GUI"""
        print(f"🎮 Launching keyboard control GUI for {self.robot_name}...")
        cmd = ["dts", "duckiebot", "keyboard_control", self.robot_name]
        
        try:
            process = subprocess.Popen(cmd)
            self.processes.append(process)
            print("✅ Keyboard control GUI launched")
            print("⚠️  Make sure the GUI window is visible and active!")
        except Exception as e:
            print(f"❌ Failed to launch keyboard GUI: {e}")
            print("   You can manually run: dts duckiebot keyboard_control " + self.robot_name)
    
    def run(self, skip_gui=False):
        """Run all components"""
        print("="*60)
        print("🚀 VLM Keyboard Control System Launcher")
        print(f"🤖 Robot: {self.robot_name}")
        print(f"💻 Laptop IP: {self.laptop_ip}")
        print("="*60)
        
        # Start components in order
        if not self.start_llama_server():
            print("❌ Cannot proceed without llama.cpp server")
            return
        
        time.sleep(2)
        
        if not self.start_keyboard_bridge():
            print("⚠️  Keyboard bridge may have issues")
        
        time.sleep(2)
        
        self.start_video_streamer()
        
        time.sleep(2)
        
        if not skip_gui:
            self.launch_keyboard_gui()
        
        print("\n" + "="*60)
        print("✅ All components launched!")
        print("\n📋 Quick Guide:")
        print("1. Open dashboard: http://localhost:8000")
        print("2. Make sure keyboard GUI is visible")
        print("3. Click 'Activate Keyboard GUI' in dashboard")
        print("4. Toggle 'Auto Control' to enable VLM control")
        print("\n⚠️  Press Ctrl+C to stop all components")
        print("="*60)
        
        # Wait for interrupt
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all processes"""
        print("\n🛑 Shutting down...")
        
        # Terminate all processes
        for process in self.processes:
            try:
                process.terminate()
            except:
                pass
        
        # Stop docker container
        try:
            subprocess.run(["docker", "stop", "llama-vlm-server"], check=False)
            subprocess.run(["docker", "rm", "llama-vlm-server"], check=False)
        except:
            pass
        
        print("✅ Cleanup complete")

def main():
    parser = argparse.ArgumentParser(description='Launch VLM Keyboard Control System')
    parser.add_argument('robot_name', help='Name of the Duckiebot')
    parser.add_argument('--laptop-ip', help='IP address of this laptop')
    parser.add_argument('--skip-gui', action='store_true', 
                       help='Skip launching keyboard GUI (launch manually)')
    
    args = parser.parse_args()
    
    launcher = VLMControlLauncher(args.robot_name, args.laptop_ip)
    
    try:
        launcher.run(skip_gui=args.skip_gui)
    except Exception as e:
        print(f"❌ Error: {e}")
        launcher.cleanup()

if __name__ == "__main__":
    main()
