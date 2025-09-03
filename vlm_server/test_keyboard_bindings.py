#!/usr/bin/env python3
"""
Test Keyboard Bindings for Duckietown VLM Control
Tests WASD key presses and provides visual feedback

This script helps verify that the VLM system is correctly sending
WASD keys to the Duckietown keyboard controller.
"""

import pyautogui
import time
import keyboard
import threading
from datetime import datetime

class KeyboardTester:
    def __init__(self):
        self.test_running = False
        self.key_press_log = []
        
        # Duckietown keyboard mappings (Arrow Keys)
        self.decision_to_key = {
            'FORWARD': 'up',
            'LEFT': 'left', 
            'RIGHT': 'right',
            'STOP': 'down'
        }
        
        print("🎮 Duckietown Keyboard Binding Tester")
        print("=" * 50)
        print("📋 Key Mappings:")
        for decision, key in self.decision_to_key.items():
            print(f"  {decision} → {key.upper()}")
        print("=" * 50)
    
    def send_key_press(self, decision):
        """Send a key press for the given decision"""
        if decision in self.decision_to_key:
            key = self.decision_to_key[decision]
            
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            try:
                # Send key press with visual feedback
                print(f"🎯 [{timestamp}] Sending: {decision} → {key.upper()}")
                
                pyautogui.keyDown(key)
                time.sleep(0.1)  # Brief press duration
                pyautogui.keyUp(key)
                
                # Log the key press
                self.key_press_log.append({
                    'timestamp': timestamp,
                    'decision': decision,
                    'key': key,
                    'status': 'SUCCESS'
                })
                
                print(f"✅ [{timestamp}] Key {key.upper()} pressed successfully")
                return True
                
            except Exception as e:
                print(f"❌ [{timestamp}] Failed to press {key.upper()}: {e}")
                self.key_press_log.append({
                    'timestamp': timestamp,
                    'decision': decision,
                    'key': key,
                    'status': f'FAILED: {e}'
                })
                return False
        else:
            print(f"❌ Unknown decision: {decision}")
            return False
    
    def test_all_keys(self):
        """Test all key bindings sequentially"""
        print("\n🚀 Testing all key bindings...")
        print("📋 Make sure the Duckietown keyboard controller window is active!")
        print("⏳ Starting test in 3 seconds...")
        
        for i in range(3, 0, -1):
            print(f"   {i}...")
            time.sleep(1)
        
        print("🎯 Starting key test sequence:")
        
        # Test sequence: Forward, Left, Right, Stop
        test_sequence = ['FORWARD', 'LEFT', 'RIGHT', 'STOP']
        
        for decision in test_sequence:
            self.send_key_press(decision)
            time.sleep(1)  # Wait between presses
        
        print("✅ Test sequence completed!")
    
    def interactive_test(self):
        """Interactive test mode"""
        print("\n🎮 Interactive Test Mode")
        print("📋 Commands:")
        print("  1 or F: Send FORWARD (↑)")
        print("  2 or L: Send LEFT (←)")
        print("  3 or R: Send RIGHT (→)")
        print("  4 or S: Send STOP (↓)")
        print("  Q: Quit")
        print("  H: Show this help")
        print("  T: Run full test sequence")
        print("  LOG: Show key press log")
        
        while True:
            try:
                command = input("\n🎯 Enter command: ").strip().upper()
                
                if command in ['Q', 'QUIT']:
                    break
                elif command in ['1', 'F', 'FORWARD']:
                    self.send_key_press('FORWARD')
                elif command in ['2', 'L', 'LEFT']:
                    self.send_key_press('LEFT')
                elif command in ['3', 'R', 'RIGHT']:
                    self.send_key_press('RIGHT')
                elif command in ['4', 'S', 'STOP']:
                    self.send_key_press('STOP')
                elif command in ['T', 'TEST']:
                    self.test_all_keys()
                elif command in ['H', 'HELP']:
                    print("\n📋 Commands:")
                    print("  1 or F: Send FORWARD (↑)")
                    print("  2 or L: Send LEFT (←)")
                    print("  3 or R: Send RIGHT (→)")
                    print("  4 or S: Send STOP (↓)")
                    print("  Q: Quit")
                    print("  T: Run full test sequence")
                    print("  LOG: Show key press log")
                elif command in ['LOG']:
                    self.show_log()
                else:
                    print(f"❌ Unknown command: {command}")
                    print("💡 Type 'H' for help")
                    
            except KeyboardInterrupt:
                break
        
        print("👋 Goodbye!")
    
    def show_log(self):
        """Show the key press log"""
        print("\n📋 Key Press Log:")
        print("-" * 60)
        if not self.key_press_log:
            print("   No key presses recorded yet")
        else:
            for entry in self.key_press_log[-10:]:  # Show last 10 entries
                timestamp = entry['timestamp']
                decision = entry['decision']
                key = entry['key'].upper()
                status = entry['status']
                print(f"   [{timestamp}] {decision} → {key} | {status}")
        print("-" * 60)
    
    def monitor_duckiebot_controller(self):
        """Help monitor if the Duckiebot keyboard controller is receiving keys"""
        print("\n📺 Duckiebot Controller Monitoring Tips:")
        print("=" * 50)
        print("1. 🎯 Make sure the keyboard controller window is ACTIVE")
        print("   Run: dts duckiebot keyboard_control ROBOT_NAME")
        print("   Click on the keyboard controller window before testing")
        print()
        print("2. 🔍 Watch for movement on your Duckiebot:")
        print("   - FORWARD (↑): Robot moves forward")
        print("   - LEFT (←): Robot turns left")
        print("   - RIGHT (→): Robot turns right") 
        print("   - STOP (↓): Robot stops/moves backward")
        print()
        print("3. 📊 Check the dashboard for feedback:")
        print("   Run: dts duckiebot dashboard ROBOT_NAME")
        print("   Look for motor commands and wheel status")
        print()
        print("4. 🐛 Debug tips:")
        print("   - Ensure robot is on and connected")
        print("   - Check battery level")
        print("   - Verify wheels are not obstructed")
        print("   - Try manual control first with keyboard")
        print("=" * 50)

def main():
    tester = KeyboardTester()
    
    print("\n📋 Choose test mode:")
    print("1. Interactive test (manual control)")
    print("2. Automatic sequence test")
    print("3. Show monitoring tips")
    print("4. Exit")
    
    while True:
        try:
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                tester.interactive_test()
                break
            elif choice == "2":
                tester.test_all_keys()
                # After auto test, show option for interactive
                cont = input("\n🎮 Continue with interactive mode? (y/n): ").strip().lower()
                if cont == 'y':
                    tester.interactive_test()
                break
            elif choice == "3":
                tester.monitor_duckiebot_controller()
                continue
            elif choice == "4":
                print("👋 Goodbye!")
                break
            else:
                print("❌ Invalid choice. Please enter 1-4.")
                
        except KeyboardInterrupt:
            print("\n👋 Goodbye!")
            break

if __name__ == "__main__":
    main()
