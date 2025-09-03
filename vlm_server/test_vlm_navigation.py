#!/usr/bin/env python3
"""
Test script for DuckieBot VLM Navigation System
Tests connectivity and basic functionality without requiring physical robot
"""

import requests
import time
import json
import base64
import cv2
import numpy as np
import argparse

def test_llama_server(url="http://localhost:8080"):
    """Test llama.cpp server connectivity and basic VLM functionality"""
    print("🧠 Testing llama.cpp server...")
    
    try:
        # Test health endpoint
        response = requests.get(f"{url}/health", timeout=5)
        if response.status_code == 200:
            print("✅ llama.cpp server is healthy")
        else:
            print(f"❌ Server health check failed: {response.status_code}")
            return False
        
        # Test VLM with sample image
        print("🖼️ Testing VLM with sample image...")
        
        # Create a simple test image (simulating camera feed)
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Draw simple road-like features
        cv2.rectangle(test_image, (200, 200), (440, 400), (50, 50, 50), -1)  # Road
        cv2.line(test_image, (320, 200), (320, 400), (255, 255, 0), 3)      # Yellow center line
        cv2.line(test_image, (210, 200), (210, 400), (255, 255, 255), 2)    # White left edge
        cv2.line(test_image, (430, 200), (430, 400), (255, 255, 255), 2)    # White right edge
        
        # Encode image
        _, buffer = cv2.imencode('.jpg', test_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        image_base64 = base64.b64encode(buffer).decode('utf-8')
        
        # Prepare VLM request
        payload = {
            "model": "Qwen2.5-VL-7B",
            "messages": [{
                "role": "user",
                "content": [
                    {
                        "type": "text", 
                        "text": "You are controlling a robot. Analyze this road image and decide: FORWARD, LEFT, RIGHT, or STOP. Respond with just the command and brief reasoning."
                    },
                    {
                        "type": "image_url", 
                        "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}
                    }
                ]
            }],
            "max_tokens": 50,
            "temperature": 0.3,
            "stream": False
        }
        
        start_time = time.time()
        response = requests.post(f"{url}/v1/chat/completions", json=payload, timeout=15)
        response_time = time.time() - start_time
        
        if response.status_code == 200:
            result = response.json()
            content = result['choices'][0]['message']['content'].strip()
            print(f"✅ VLM responded in {response_time:.2f}s: {content}")
            return True
        else:
            print(f"❌ VLM request failed: {response.status_code}")
            return False
            
    except requests.exceptions.Timeout:
        print("❌ Request timed out")
        return False
    except Exception as e:
        print(f"❌ Test failed: {e}")
        return False

def test_ros_topics(robot_name):
    """Test ROS topic availability (requires ROS environment)"""
    print("📡 Testing ROS topics...")
    
    try:
        import subprocess
        
        # Check if ROS is running
        result = subprocess.run(["rostopic", "list"], capture_output=True, text=True, timeout=5)
        
        if result.returncode != 0:
            print("❌ ROS not running or not accessible")
            return False
        
        topics = result.stdout.strip().split('\n')
        
        # Check for camera topic
        camera_topic = f"/{robot_name}/camera_node/image/compressed"
        if camera_topic in topics:
            print(f"✅ Camera topic found: {camera_topic}")
        else:
            print(f"⚠️ Camera topic not found: {camera_topic}")
            print("Available topics:")
            for topic in topics[:10]:  # Show first 10 topics
                if 'camera' in topic.lower():
                    print(f"  {topic}")
        
        # Check for control topics
        control_topics = [
            f"/{robot_name}/wheels_driver_node/wheels_cmd",
            f"/{robot_name}/car_cmd_switch_node/cmd"
        ]
        
        for topic in control_topics:
            if topic in topics:
                print(f"✅ Control topic found: {topic}")
            else:
                print(f"⚠️ Control topic not found: {topic}")
        
        return True
        
    except ImportError:
        print("⚠️ ROS Python packages not available")
        return False
    except subprocess.TimeoutExpired:
        print("❌ ROS topic check timed out")
        return False
    except Exception as e:
        print(f"❌ ROS test failed: {e}")
        return False

def test_network_connectivity(robot_name):
    """Test network connectivity to DuckieBot"""
    print(f"🌐 Testing connectivity to {robot_name}...")
    
    try:
        import subprocess
        
        # Try to ping the robot
        result = subprocess.run(
            ["ping", "-c", "3", f"{robot_name}.local"], 
            capture_output=True, 
            text=True, 
            timeout=10
        )
        
        if result.returncode == 0:
            print(f"✅ {robot_name}.local is reachable")
            return True
        else:
            print(f"❌ Cannot reach {robot_name}.local")
            print("Make sure:")
            print(f"  1. DuckieBot {robot_name} is powered on")
            print("  2. Robot is connected to the same network")
            print("  3. Hostname is correct")
            return False
            
    except subprocess.TimeoutExpired:
        print(f"❌ Ping to {robot_name}.local timed out")
        return False
    except Exception as e:
        print(f"❌ Network test failed: {e}")
        return False

def run_integration_test(robot_name, llama_url):
    """Run a basic integration test"""
    print("🔧 Running integration test...")
    
    # This would test the actual navigation system
    # For now, just verify all components are ready
    
    tests = [
        ("VLM Server", lambda: test_llama_server(llama_url)),
        ("Network", lambda: test_network_connectivity(robot_name)),
        ("ROS Topics", lambda: test_ros_topics(robot_name)),
    ]
    
    results = {}
    for test_name, test_func in tests:
        print(f"\n--- {test_name} Test ---")
        results[test_name] = test_func()
    
    print("\n" + "="*50)
    print("🧪 TEST RESULTS SUMMARY")
    print("="*50)
    
    all_passed = True
    for test_name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{test_name:15} {status}")
        if not passed:
            all_passed = False
    
    print("="*50)
    
    if all_passed:
        print("✅ All tests passed! System ready for navigation.")
        print(f"\nTo start navigation:")
        print(f"  python start_vlm_navigation.py {robot_name}")
    else:
        print("❌ Some tests failed. Check the issues above.")
        print("\nCommon solutions:")
        print("  - Start llama.cpp server: see README")
        print("  - Check robot connection: ping <robot>.local")
        print("  - Source ROS environment: source devel/setup.bash")
    
    return all_passed

def main():
    parser = argparse.ArgumentParser(description='Test DuckieBot VLM Navigation System')
    parser.add_argument('robot_name', help='Name of the DuckieBot')
    parser.add_argument('--llama-url', default='http://localhost:8080',
                       help='URL of llama.cpp server')
    parser.add_argument('--test-only', choices=['vlm', 'network', 'ros'],
                       help='Run only specific test')
    
    args = parser.parse_args()
    
    print("🧪 DuckieBot VLM Navigation System Test")
    print("="*50)
    print(f"Robot: {args.robot_name}")
    print(f"VLM Server: {args.llama_url}")
    print("="*50)
    
    if args.test_only:
        if args.test_only == 'vlm':
            test_llama_server(args.llama_url)
        elif args.test_only == 'network':
            test_network_connectivity(args.robot_name)
        elif args.test_only == 'ros':
            test_ros_topics(args.robot_name)
    else:
        run_integration_test(args.robot_name, args.llama_url)

if __name__ == '__main__':
    main()
