#!/usr/bin/env python3
"""
VLM Mission Control Utility with Gemma 3 Support

Enhanced with support for switching between LLaVA and Gemma 3 models.
Inspired by the YouTube video analysis and Hugging Face Gemma 3 blog post.

Usage:
    python3 vlm_mission_control.py --robot duckiebot01 --mission explore
    python3 vlm_mission_control.py --robot duckiebot01 --model gemma3-4b --fast
    python3 vlm_mission_control.py --robot duckiebot01 --models
"""

import argparse
import sys
import time
import requests
import rospy
from std_msgs.msg import String

class VLMMissionControl:
    def __init__(self, robot_name, laptop_ip="192.168.1.100", laptop_port=5000):
        self.robot_name = robot_name
        self.laptop_ip = laptop_ip
        self.laptop_port = laptop_port
        
        # Server endpoints
        self.mission_url = f"http://{laptop_ip}:{laptop_port}/set_mission"
        self.performance_url = f"http://{laptop_ip}:{laptop_port}/performance"
        self.status_url = f"http://{laptop_ip}:{laptop_port}/gui_status"
        self.model_switch_url = f"http://{laptop_ip}:{laptop_port}/switch_model"
        self.available_models_url = f"http://{laptop_ip}:{laptop_port}/available_models"
        
        # ROS topics
        self.mode_topic = f"/{robot_name}/operation_mode"
        self.mission_topic = f"/{robot_name}/vlm_mission"
        
        # Initialize ROS node
        try:
            rospy.init_node('vlm_mission_control', anonymous=True)
            self.mode_pub = rospy.Publisher(self.mode_topic, String, queue_size=1)
            self.mission_pub = rospy.Publisher(self.mission_topic, String, queue_size=1)
            rospy.sleep(1)  # Allow publishers to connect
            print(f"✅ Connected to robot: {robot_name}")
        except Exception as e:
            print(f"❌ Failed to connect to ROS: {e}")
            print("Make sure roscore is running and you're in the correct ROS environment")
            sys.exit(1)

    def switch_model(self, model_name):
        """Switch VLM model on server"""
        try:
            response = requests.post(self.model_switch_url, json={"model": model_name}, timeout=10)
            if response.status_code == 200:
                result = response.json()
                print(f"✅ Model switched to: {model_name}")
                print(f"   Full model name: {result.get('model_name', 'Unknown')}")
                return True
            else:
                error_detail = response.json().get('detail', 'Unknown error')
                print(f"❌ Failed to switch model: {error_detail}")
                return False
        except Exception as e:
            print(f"❌ Could not reach VLM server: {e}")
            return False

    def get_available_models(self):
        """Get available models from server"""
        try:
            response = requests.get(self.available_models_url, timeout=5)
            if response.status_code == 200:
                return response.json()
            else:
                print(f"❌ Server responded with status {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ Could not reach VLM server: {e}")
            return None

    def print_available_models(self):
        """Print formatted list of available models"""
        models_info = self.get_available_models()
        if not models_info:
            return
        
        print("\n🤖 AVAILABLE VLM MODELS")
        print("=" * 60)
        print(f"Current Model: {models_info.get('current_model', 'Unknown')}")
        print("\nModel Status:")
        
        for key, info in models_info.get('available_models', {}).items():
            status = "✅ Ready" if info['available'] else "❌ Not Downloaded"
            model_name = info['name']
            print(f"  {key:12} | {model_name:20} | {status}")
        
        print("\n📋 RECOMMENDATIONS:")
        recommendations = models_info.get('recommended', {})
        for hardware, model in recommendations.items():
            print(f"  {hardware:15}: {model}")
        
        print("\n💾 TO DOWNLOAD MODELS:")
        for key, info in models_info.get('available_models', {}).items():
            if not info['available']:
                print(f"  ollama pull {info['name']}")

    def set_mode(self, mode):
        """Switch between 'map' and 'vlm' modes"""
        valid_modes = ['map', 'vlm']
        if mode not in valid_modes:
            print(f"❌ Invalid mode. Valid modes: {valid_modes}")
            return False
        
        try:
            msg = String()
            msg.data = mode
            self.mode_pub.publish(msg)
            print(f"✅ Mode switched to: {mode.upper()}")
            return True
        except Exception as e:
            print(f"❌ Failed to switch mode: {e}")
            return False

    def set_mission(self, mission):
        """Set VLM mission on both server and robot"""
        valid_missions = ['explore', 'find_books', 'find_friends', 'navigate', 'clean']
        if mission not in valid_missions:
            print(f"❌ Invalid mission. Valid missions: {valid_missions}")
            return False
        
        success = True
        
        # Set mission on VLM server
        try:
            response = requests.post(self.mission_url, params={"mission": mission}, timeout=5)
            if response.status_code == 200:
                result = response.json()
                current_model = result.get('current_model', 'Unknown')
                print(f"✅ Mission set on VLM server: {mission} (Model: {current_model})")
            else:
                print(f"⚠️ Server responded with status {response.status_code}")
                success = False
        except Exception as e:
            print(f"⚠️ Could not reach VLM server: {e}")
            success = False
        
        # Set mission via ROS topic
        try:
            msg = String()
            msg.data = mission
            self.mission_pub.publish(msg)
            print(f"✅ Mission sent to robot: {mission}")
        except Exception as e:
            print(f"❌ Failed to send mission to robot: {e}")
            success = False
        
        return success

    def get_performance_stats(self):
        """Get performance statistics from VLM server"""
        try:
            response = requests.get(self.performance_url, timeout=5)
            if response.status_code == 200:
                stats = response.json()
                return stats
            else:
                print(f"❌ Server responded with status {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ Could not reach VLM server: {e}")
            return None

    def get_system_status(self):
        """Get current system status from VLM server"""
        try:
            response = requests.get(self.status_url, timeout=5)
            if response.status_code == 200:
                status = response.json()
                return status
            else:
                print(f"❌ Server responded with status {response.status_code}")
                return None
        except Exception as e:
            print(f"❌ Could not reach VLM server: {e}")
            return None

    def print_performance_stats(self):
        """Print formatted performance statistics with model information"""
        stats = self.get_performance_stats()
        if not stats:
            return
        
        print("\n📊 PERFORMANCE STATISTICS")
        print("=" * 50)
        print(f"Model Type: {stats.get('model_type', 'Unknown')}")
        print(f"Full Model: {stats.get('model', 'Unknown')}")
        print(f"Last Processing Time: {stats.get('last_processing_time', 0):.2f}s")
        print(f"Estimated FPS: {stats.get('estimated_fps', 0):.2f}")
        print(f"Fast Mode: {'ENABLED' if stats.get('fast_mode_enabled', False) else 'DISABLED'}")
        print(f"Current Mission: {stats.get('current_mission', 'Unknown').upper()}")
        
        # Performance rating with model-specific expectations
        fps = stats.get('estimated_fps', 0)
        model_type = stats.get('model_type', '')
        
        if model_type.startswith('gemma3'):
            if fps >= 1.5:
                rating = "🟢 EXCELLENT (>1.5 FPS - Gemma 3 optimized)"
            elif fps >= 1.0:
                rating = "🟢 VERY GOOD (>1 FPS - good Gemma 3 performance)"
            elif fps >= 0.5:
                rating = "🟡 ACCEPTABLE (0.5-1 FPS)"
            else:
                rating = "🔴 SLOW (<0.5 FPS - consider switching models)"
        else:  # LLaVA models
            if fps >= 1.0:
                rating = "🟢 EXCELLENT (>1 FPS - matches video performance)"
            elif fps >= 0.5:
                rating = "🟡 GOOD (0.5-1 FPS)"
            else:
                rating = "🔴 SLOW (<0.5 FPS)"
        
        print(f"Performance Rating: {rating}")
        
        # Model recommendations
        recommendations = stats.get('recommended_for_hardware', {})
        if recommendations:
            print(f"\n💡 RECOMMENDATIONS:")
            for hardware, model in recommendations.items():
                current = "👈 CURRENT" if model == stats.get('model_type') else ""
                print(f"  {hardware}: {model} {current}")

    def print_system_status(self):
        """Print formatted system status"""
        status = self.get_system_status()
        if not status:
            return
        
        print("\n🤖 SYSTEM STATUS")
        print("=" * 50)
        print(f"Last Update: {status.get('timestamp', 'Unknown')}")
        print(f"Current Model: {status.get('current_model', 'Unknown')}")
        print(f"Fast Mode: {'ENABLED' if status.get('fast_mode', False) else 'DISABLED'}")
        
        if status.get('vlm_raw_output'):
            print(f"Last VLM Response: {status.get('vlm_raw_output', '')[:100]}...")
        
        if status.get('command_sent'):
            cmd = status.get('command_sent', {})
            print(f"Last Command: {cmd.get('action', 'Unknown').upper()}")
            print(f"Model Used: {cmd.get('model_used', 'Unknown')}")
            if cmd.get('reasoning'):
                print(f"Reasoning: {cmd.get('reasoning', '')[:100]}...")
        
        if status.get('error'):
            print(f"❌ Error: {status.get('error')}")

    def monitor_realtime(self, duration=60):
        """Monitor system performance in real-time"""
        print(f"\n🔄 MONITORING {self.robot_name} for {duration} seconds...")
        print("Press Ctrl+C to stop monitoring")
        print("=" * 60)
        
        start_time = time.time()
        last_stats_time = 0
        
        try:
            while time.time() - start_time < duration:
                current_time = time.time()
                
                # Print stats every 5 seconds
                if current_time - last_stats_time >= 5:
                    stats = self.get_performance_stats()
                    if stats:
                        fps = stats.get('estimated_fps', 0)
                        processing_time = stats.get('last_processing_time', 0)
                        mission = stats.get('current_mission', 'unknown')
                        fast_mode = 'FAST' if stats.get('fast_mode_enabled', False) else 'DETAILED'
                        model_type = stats.get('model_type', 'unknown')
                        
                        elapsed = int(current_time - start_time)
                        print(f"[{elapsed:03d}s] FPS: {fps:.2f} | Time: {processing_time:.2f}s | Mode: {fast_mode} | Model: {model_type} | Mission: {mission.upper()}")
                    
                    last_stats_time = current_time
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n\n⏹️ Monitoring stopped by user")

def main():
    parser = argparse.ArgumentParser(description="VLM Mission Control for DuckieBot with Gemma 3 Support")
    parser.add_argument('--robot', '-r', required=True, help="Robot name (e.g., duckiebot01)")
    parser.add_argument('--laptop-ip', default="192.168.1.100", help="Laptop IP address")
    parser.add_argument('--laptop-port', type=int, default=5000, help="Laptop server port")
    
    # Actions
    parser.add_argument('--mode', choices=['map', 'vlm'], help="Switch operation mode")
    parser.add_argument('--mission', choices=['explore', 'find_books', 'find_friends', 'navigate', 'clean'], 
                       help="Set VLM mission")
    parser.add_argument('--model', choices=['gemma3-4b', 'gemma3-12b', 'llava-7b', 'llava-13b'], 
                       help="Switch VLM model")
    parser.add_argument('--models', action='store_true', help="Show available models")
    parser.add_argument('--stats', action='store_true', help="Show performance statistics")
    parser.add_argument('--status', action='store_true', help="Show system status")
    parser.add_argument('--monitor', type=int, metavar='SECONDS', help="Monitor performance for N seconds")
    
    # Quick presets (updated with model options)
    parser.add_argument('--fast', action='store_true', help="Quick setup: VLM mode + fast exploration")
    parser.add_argument('--gemma3', action='store_true', help="Quick setup: Switch to Gemma 3 4B + fast mode")
    parser.add_argument('--gemma3-12b', action='store_true', help="Quick setup: Switch to Gemma 3 12B + detailed mode")
    parser.add_argument('--books', action='store_true', help="Quick setup: VLM mode + find books mission")
    parser.add_argument('--friends', action='store_true', help="Quick setup: VLM mode + find friends mission")
    
    args = parser.parse_args()
    
    # Create controller
    controller = VLMMissionControl(args.robot, args.laptop_ip, args.laptop_port)
    
    # Handle quick presets
    if args.gemma3:
        print("🚀 GEMMA 3 FAST SETUP")
        controller.switch_model('gemma3-4b')
        time.sleep(2)
        controller.set_mode('vlm')
        controller.set_mission('explore')
        time.sleep(1)
        controller.print_performance_stats()
        return
    
    if args.gemma3_12b:
        print("🧠 GEMMA 3 12B DETAILED SETUP")
        controller.switch_model('gemma3-12b')
        time.sleep(2)
        controller.set_mode('vlm')
        controller.set_mission('explore')
        time.sleep(1)
        controller.print_performance_stats()
        return
    
    if args.fast:
        print("🚀 FAST EXPLORATION SETUP")
        controller.set_mode('vlm')
        controller.set_mission('explore')
        time.sleep(1)
        controller.print_performance_stats()
        return
    
    if args.books:
        print("📚 BOOK FINDING SETUP")
        controller.set_mode('vlm')
        controller.set_mission('find_books')
        time.sleep(1)
        controller.print_performance_stats()
        return
    
    if args.friends:
        print("🤖 FRIEND FINDING SETUP")
        controller.set_mode('vlm')
        controller.set_mission('find_friends')
        time.sleep(1)
        controller.print_performance_stats()
        return
    
    # Handle individual actions
    if args.model:
        controller.switch_model(args.model)
        time.sleep(1)
    
    if args.mode:
        controller.set_mode(args.mode)
    
    if args.mission:
        controller.set_mission(args.mission)
    
    if args.models:
        controller.print_available_models()
    
    if args.stats:
        controller.print_performance_stats()
    
    if args.status:
        controller.print_system_status()
    
    if args.monitor:
        controller.monitor_realtime(args.monitor)
    
    # If no action specified, show help
    if not any([args.mode, args.mission, args.model, args.models, args.stats, args.status, args.monitor]):
        print("🎮 VLM Mission Control Ready! (Now with Gemma 3 Support)")
        print("\nQuick Commands:")
        print(f"  python3 vlm_mission_control.py --robot {args.robot} --gemma3")
        print(f"  python3 vlm_mission_control.py --robot {args.robot} --gemma3-12b")
        print(f"  python3 vlm_mission_control.py --robot {args.robot} --models")
        print(f"  python3 vlm_mission_control.py --robot {args.robot} --model gemma3-4b --fast")
        print(f"  python3 vlm_mission_control.py --robot {args.robot} --stats")
        print("\nFor more options, use --help")

if __name__ == "__main__":
    main() 