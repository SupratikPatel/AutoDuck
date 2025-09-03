#!/usr/bin/env python3
"""
Example Usage of VLM Keyboard Control System

This demonstrates how to use the VLM system to control your Duckiebot
"""

import time
import requests
import base64
import cv2
import numpy as np

def test_vlm_control():
    """Test the VLM control system with a sample image"""
    
    print("🧪 Testing VLM Keyboard Control System")
    
    # 1. Check if services are running
    print("\n1️⃣ Checking services...")
    
    # Check llama.cpp server
    try:
        response = requests.get("http://localhost:8080/health", timeout=2)
        print("✅ llama.cpp server is running")
    except:
        print("❌ llama.cpp server not found. Start it with:")
        print("   docker run --gpus all -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda \\")
        print("       -hf ggml-org/Qwen2.5-VL-7B-Instruct-GGUF \\")
        print("       --host 0.0.0.0 --port 8080 --n-gpu-layers 99")
        return
    
    # Check keyboard bridge
    try:
        response = requests.get("http://localhost:8000/control/status", timeout=2)
        print("✅ Keyboard control bridge is running")
        status = response.json()
        print(f"   Auto control: {status['auto_control']}")
        print(f"   Keyboard GUI active: {status['keyboard_gui_active']}")
    except:
        print("❌ Keyboard bridge not found. Start it with:")
        print("   python keyboard_control_bridge.py")
        return
    
    # 2. Create a test image (simulated road view)
    print("\n2️⃣ Creating test image...")
    test_image = create_test_road_image()
    
    # Encode image to base64
    _, buffer = cv2.imencode('.jpg', test_image)
    image_base64 = base64.b64encode(buffer).decode('utf-8')
    
    # 3. Send test frame to VLM
    print("\n3️⃣ Sending test frame to VLM...")
    
    payload = {
        "image_base64": image_base64,
        "timestamp": time.time(),
        "robot_name": "test_robot"
    }
    
    try:
        response = requests.post("http://localhost:8000/process_frame", json=payload, timeout=15)
        if response.status_code == 200:
            result = response.json()
            print(f"✅ VLM Decision: {result.get('decision', 'UNKNOWN')}")
            print(f"   Reasoning: {result.get('reasoning', 'No reasoning provided')}")
            print(f"   Processing time: {result.get('processing_time', 0):.2f}s")
            print(f"   Auto-executed: {result.get('auto_executed', False)}")
        else:
            print(f"❌ Error: {response.status_code}")
    except Exception as e:
        print(f"❌ Request failed: {e}")
    
    # 4. Test manual control
    print("\n4️⃣ Testing manual control commands...")
    
    commands = [
        ("FORWARD", "w", 0.1),
        ("LEFT", "a", 0.1),
        ("RIGHT", "d", 0.1),
        ("STOP", "s", 0.1)
    ]
    
    for name, key, duration in commands:
        print(f"   Sending {name} command...")
        try:
            response = requests.post(
                "http://localhost:8000/control/manual",
                json={"command": key, "duration": duration}
            )
            if response.status_code == 200:
                print(f"   ✅ {name} command sent")
            time.sleep(0.5)
        except:
            print(f"   ❌ Failed to send {name} command")
    
    # 5. Get final statistics
    print("\n5️⃣ Getting performance statistics...")
    try:
        response = requests.get("http://localhost:8000/stats")
        if response.status_code == 200:
            stats = response.json()
            print(f"   Total frames: {stats['total_frames']}")
            print(f"   Processed frames: {stats['processed_frames']}")
            print(f"   Success rate: {stats['successful_decisions']}/{stats['processed_frames']}")
            print(f"   Average FPS: {stats['current_fps']:.2f}")
    except:
        print("   ❌ Could not get statistics")
    
    print("\n✅ Test complete!")
    print("📱 Open http://localhost:8000 to see the dashboard")

def create_test_road_image():
    """Create a simulated road view for testing"""
    # Create blank image
    img = np.ones((480, 640, 3), dtype=np.uint8) * 128  # Gray background
    
    # Draw road
    road_color = (80, 80, 80)  # Dark gray
    cv2.rectangle(img, (160, 0), (480, 480), road_color, -1)
    
    # Draw lane lines
    yellow = (0, 255, 255)  # Yellow line (left)
    white = (255, 255, 255)  # White line (right)
    
    # Left yellow line
    cv2.line(img, (200, 480), (250, 0), yellow, 5)
    
    # Right white line  
    cv2.line(img, (440, 480), (390, 0), white, 5)
    
    # Add some text
    cv2.putText(img, "Test Road View", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(img, "Clear path ahead", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
    
    return img

def quick_start_guide():
    """Print quick start guide"""
    print("🚀 VLM Keyboard Control - Quick Start Guide")
    print("=" * 50)
    print("\n1. Start llama.cpp server:")
    print("   cd vlm_server")
    print("   ./start_llama_server.sh")
    print("\n2. Start keyboard control bridge:")
    print("   python keyboard_control_bridge.py")
    print("\n3. Launch Duckiebot keyboard GUI:")
    print("   dts duckiebot keyboard_control YOUR_ROBOT_NAME")
    print("\n4. Start video streamer:")
    print("   python duckiebot_vlm_streamer.py YOUR_ROBOT_NAME")
    print("\n5. Open dashboard:")
    print("   http://localhost:8000")
    print("\nOr use the integrated launcher:")
    print("   python launch_vlm_keyboard_control.py YOUR_ROBOT_NAME")
    print("=" * 50)

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "--guide":
        quick_start_guide()
    else:
        test_vlm_control()
