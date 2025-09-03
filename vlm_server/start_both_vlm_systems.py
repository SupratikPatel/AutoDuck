#!/usr/bin/env python3
"""
Start Both VLM Systems Together
Launches llama.cpp server and both dashboard systems

URLs:
- llama.cpp server: http://localhost:8080 (VLM processing)
- llamacpp_autoduck: http://localhost:5000 (webcam VLM with dashboard)
- screen_capture_vlm: http://localhost:3000 (screen capture VLM with dashboard)
"""

import subprocess
import time
import requests
import sys
import os
import threading

def check_docker():
    """Check if Docker is available"""
    try:
        subprocess.run(["docker", "--version"], check=True, capture_output=True)
        return True
    except:
        print("❌ Docker not found. Please install Docker first.")
        return False

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

def run_llamacpp_autoduck():
    """Run the original llamacpp_autoduck.py system"""
    print("🎥 Starting llamacpp_autoduck.py (webcam VLM)")
    try:
        subprocess.run([sys.executable, "llamacpp/llamacpp_autoduck.py"])
    except Exception as e:
        print(f"❌ Error running llamacpp_autoduck: {e}")

def run_screen_capture_vlm():
    """Run the new screen capture VLM with dashboard"""
    print("📺 Starting screen_capture_vlm_with_dashboard.py (VLM Dashboard)")
    try:
        subprocess.run([sys.executable, "screen_capture_vlm_with_dashboard.py"])
    except Exception as e:
        print(f"❌ Error running screen capture VLM: {e}")

def main():
    print("🚀 Starting Both VLM Systems")
    print("="*60)
    print("📊 URLs after startup:")
    print("  - llama.cpp server: http://localhost:8080")
    print("  - llamacpp_autoduck: http://localhost:5000 (webcam)")
    print("  - screen_capture_vlm: http://localhost:3000 (screen capture)")
    print("="*60)
    
    # Check prerequisites
    if not check_docker():
        return
    
    # Start llama.cpp server
    if not start_llama_server():
        return
    
    print("\n📋 Choose which system to start:")
    print("1. llamacpp_autoduck.py (webcam VLM) - localhost:5000")
    print("2. screen_capture_vlm_with_dashboard.py (VLM Dashboard) - localhost:3000")
    print("3. Both systems (in separate terminals)")
    print("4. Exit")
    
    choice = input("\nEnter choice (1-4): ").strip()
    
    if choice == "1":
        print("\n🎥 Starting webcam VLM system...")
        print("📊 Dashboard will be at: http://localhost:5000")
        run_llamacpp_autoduck()
        
    elif choice == "2":
        print("\n📺 Starting screen capture VLM system...")
        print("📊 Dashboard will be at: http://localhost:3000")
        print("📋 Make sure to have:")
        print("  - dts start_gui_tools ROBOT_NAME")
        print("  - rqt_image_view (select camera_node/image/compressed)")
        print("  - dts duckiebot keyboard_control ROBOT_NAME")
        run_screen_capture_vlm()
        
    elif choice == "3":
        print("\n🔄 Starting both systems...")
        print("📊 Dashboards will be at:")
        print("  - Webcam VLM: http://localhost:5000")
        print("  - Screen Capture VLM: http://localhost:3000")
        
        # Start llamacpp_autoduck in background thread
        autoduck_thread = threading.Thread(target=run_llamacpp_autoduck, daemon=True)
        autoduck_thread.start()
        
        time.sleep(2)  # Give first system time to start
        
        # Start screen capture VLM in main thread
        run_screen_capture_vlm()
        
    elif choice == "4":
        print("👋 Exiting...")
        return
        
    else:
        print("❌ Invalid choice")
        return

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    except Exception as e:
        print(f"❌ Error: {e}")
