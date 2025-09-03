#!/usr/bin/env python3
"""
Quick starter for Laptop VLM components
Handles llama.cpp server startup and screen capture VLM
"""

import subprocess
import time
import requests
import sys
import os

def check_docker():
    """Check if Docker is available"""
    try:
        subprocess.run(["docker", "--version"], check=True, capture_output=True)
        return True
    except:
        print("❌ Docker not found. Please install Docker first.")
        return False

def check_gpu():
    """Check if NVIDIA GPU is available"""
    try:
        result = subprocess.run(["docker", "run", "--rm", "--gpus", "all", "nvidia/cuda:11.8-base-ubuntu20.04", "nvidia-smi"], 
                              capture_output=True, text=True, timeout=10)
        return result.returncode == 0
    except:
        print("⚠️  GPU check failed, will try anyway...")
        return True

def check_llama_server():
    """Check if llama.cpp server is running"""
    try:
        response = requests.get("http://localhost:8080/health", timeout=2)
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
        for i in range(30):
            if check_llama_server():
                print("✅ llama.cpp server started successfully")
                return True
            time.sleep(2)
            print(".", end="", flush=True)
        
        print("\n❌ Server failed to start in 60 seconds")
        return False
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to start server: {e}")
        return False

def check_dependencies():
    """Check Python dependencies"""
    required = ["mss", "pygetwindow", "pyautogui", "keyboard", "cv2", "requests", "numpy"]
    missing = []
    
    for pkg in required:
        try:
            if pkg == "cv2":
                import cv2
            else:
                __import__(pkg)
        except ImportError:
            missing.append(pkg)
    
    if missing:
        print(f"❌ Missing dependencies: {missing}")
        print("Install with: pip install mss pygetwindow pyautogui keyboard opencv-python-headless requests numpy")
        return False
    
    return True

def main():
    print("🚀 Starting Laptop VLM Components")
    print("="*50)
    
    # Check prerequisites
    if not check_docker():
        return
    
    if not check_gpu():
        print("⚠️  Continuing without GPU verification...")
    
    if not check_dependencies():
        return
    
    # Start llama.cpp server
    if not start_llama_server():
        return
    
    print("\n📋 Next steps:")
    print("1. Make sure Duckiebot camera GUI is running:")
    print("   dts start_gui_tools ROBOT_NAME")
    print("   rqt_image_view → select camera_node/image/compressed")
    print("")
    print("2. Make sure keyboard control is running:")
    print("   dts duckiebot keyboard_control ROBOT_NAME")
    print("")
    print("3. Run screen capture VLM:")
    print("   python simple_screen_capture_vlm.py")
    print("")
    
    # Ask if user wants to start screen capture VLM
    if input("Start screen capture VLM now? (y/n): ").lower() == 'y':
        print("\n🎥 Starting screen capture VLM...")
        try:
            subprocess.run([sys.executable, "simple_screen_capture_vlm.py"])
        except KeyboardInterrupt:
            print("\n✅ VLM stopped by user")
        except Exception as e:
            print(f"❌ Error starting VLM: {e}")
    
    print("\n✅ Setup complete!")

if __name__ == "__main__":
    main()
