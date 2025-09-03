#!/usr/bin/env python3
"""
Test VLM Keyboard Output with Notepad
Quick test to verify keyboard simulation is working by typing in a text editor
"""

import pyautogui
import time
import subprocess
import os

def test_notepad_typing():
    print("🧪 Testing VLM Keyboard Output with Notepad")
    print("=" * 50)
    
    try:
        # Open Notepad
        print("📝 Opening Notepad...")
        if os.name == 'nt':  # Windows
            subprocess.Popen(['notepad.exe'])
        else:  # Linux/Mac
            subprocess.Popen(['gedit'])  # or 'nano', 'vim', etc.
        
        time.sleep(2)  # Wait for notepad to open
        
        print("✅ Notepad opened!")
        print("\n🎯 Testing WASD key simulation...")
        print("📋 The following should appear in Notepad:")
        
        # Test sequence
        test_keys = [
            ('W', 'FORWARD'),
            ('A', 'LEFT'), 
            ('S', 'STOP'),
            ('D', 'RIGHT'),
            ('W', 'FORWARD'),
            ('W', 'FORWARD'),
            ('S', 'STOP')
        ]
        
        for key, action in test_keys:
            print(f"  🎮 Sending: {action} → {key}")
            
            # Type the key in notepad
            pyautogui.press(key.lower())
            time.sleep(0.5)
            
            # Add space for readability
            pyautogui.press('space')
            time.sleep(0.3)
        
        # Add newline and timestamp
        pyautogui.press('enter')
        timestamp = time.strftime("%H:%M:%S")
        pyautogui.write(f"Test completed at {timestamp}")
        
        print("\n✅ Test completed!")
        print("📋 You should see: 'w a s d w w s' in Notepad")
        print("💡 This confirms keyboard simulation is working!")
        
    except Exception as e:
        print(f"❌ Error during test: {e}")

def test_continuous_simulation():
    print("\n🔄 Continuous Typing Test")
    print("=" * 30)
    print("📋 This simulates what the VLM system does:")
    print("   1. Analyze screen content")
    print("   2. Make decision")  
    print("   3. Send keyboard command")
    print("   4. Repeat...")
    
    input("\n⏳ Press ENTER to start continuous test (Ctrl+C to stop)...")
    
    try:
        decisions = ['FORWARD', 'LEFT', 'RIGHT', 'STOP']
        keys = ['w', 'a', 'd', 's']
        
        print("🚀 Starting continuous typing simulation...")
        print("📝 Make sure Notepad is the active window!")
        
        count = 0
        while True:
            decision = decisions[count % 4]
            key = keys[count % 4]
            
            print(f"🤖 Decision: {decision} → Pressing {key.upper()}")
            
            # Simulate VLM decision + keyboard press
            pyautogui.press(key)
            pyautogui.press('space')
            
            count += 1
            time.sleep(1.5)  # Similar to VLM response time
            
            if count >= 20:  # Stop after 20 presses
                break
                
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
    
    print("✅ Continuous test completed!")

def main():
    print("🎮 VLM Keyboard Simulation Tester")
    print("=" * 50)
    print("📋 Test Options:")
    print("1. Quick WASD test in Notepad")
    print("2. Continuous typing simulation") 
    print("3. Both tests")
    print("4. Exit")
    
    while True:
        try:
            choice = input("\n🎯 Enter choice (1-4): ").strip()
            
            if choice == "1":
                test_notepad_typing()
                break
            elif choice == "2":
                test_continuous_simulation()
                break
            elif choice == "3":
                test_notepad_typing()
                input("\n⏳ Press ENTER to continue to continuous test...")
                test_continuous_simulation()
                break
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
