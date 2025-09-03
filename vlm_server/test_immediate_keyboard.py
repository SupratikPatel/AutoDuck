#!/usr/bin/env python3
"""
Immediate Keyboard Test - Direct WASD Simulation
This script immediately sends WASD keys to whatever window is active (like Notepad)
to prove that keyboard simulation is working.
"""

import pyautogui
import time
import subprocess
import os

def open_notepad():
    """Open Notepad for testing"""
    try:
        if os.name == 'nt':  # Windows
            subprocess.Popen(['notepad.exe'])
            print("📝 Notepad opened! Click on it to make it active.")
        else:  # Linux/Mac  
            subprocess.Popen(['gedit'])
            print("📝 Text editor opened! Click on it to make it active.")
        
        time.sleep(3)  # Give time to focus the window
        return True
    except Exception as e:
        print(f"❌ Could not open text editor: {e}")
        return False

def send_vlm_decision(decision):
    """Send a VLM decision as keyboard input"""
    # VLM decision to WASD mapping (same as in VLM systems)
    decision_to_key = {
        'FORWARD': 'w',
        'LEFT': 'a', 
        'RIGHT': 'd',
        'STOP': 's'
    }
    
    if decision in decision_to_key:
        key = decision_to_key[decision]
        
        print(f"🤖 VLM Decision: {decision} → Pressing {key.upper()}")
        
        # Send key press (same method as VLM systems)
        pyautogui.keyDown(key)
        time.sleep(0.1)
        pyautogui.keyUp(key)
        
        # Add space for readability
        pyautogui.press('space')
        
        return True
    else:
        print(f"❌ Unknown decision: {decision}")
        return False

def test_immediate_keyboard():
    """Test immediate keyboard output"""
    print("🚀 IMMEDIATE KEYBOARD TEST")
    print("=" * 50)
    print("🎯 This simulates exactly what the VLM system does:")
    print("   1. VLM makes decision (FORWARD/LEFT/RIGHT/STOP)")
    print("   2. Decision mapped to WASD key")
    print("   3. Key sent to active window")
    print("=" * 50)
    
    # Open Notepad
    if not open_notepad():
        print("⚠️ Please open Notepad manually and click on it")
        input("Press ENTER when Notepad is ready...")
    
    print("\n🎮 Sending VLM decisions as keyboard input...")
    print("📝 You should see WASD characters appear in Notepad!")
    
    # Test sequence - typical VLM decisions
    decisions = [
        'FORWARD', 'FORWARD', 'LEFT', 'FORWARD', 
        'RIGHT', 'FORWARD', 'STOP', 'FORWARD',
        'LEFT', 'LEFT', 'FORWARD', 'RIGHT'
    ]
    
    print(f"\n⏳ Starting in 3 seconds... (sending {len(decisions)} decisions)")
    for i in range(3, 0, -1):
        print(f"   {i}...")
        time.sleep(1)
    
    print("\n🚀 Starting keyboard simulation:")
    
    for i, decision in enumerate(decisions, 1):
        send_vlm_decision(decision)
        print(f"   [{i}/{len(decisions)}] Sent: {decision}")
        time.sleep(0.8)  # Similar timing to VLM
    
    # Add timestamp
    pyautogui.press('enter')
    timestamp = time.strftime("%H:%M:%S")
    pyautogui.write(f"Test completed at {timestamp}")
    
    print("\n✅ Test completed!")
    print("📋 Expected in Notepad: 'w w a w d w s w a a w d'")
    print("💡 If you see these characters, keyboard simulation is working!")

def test_continuous_simulation():
    """Test continuous keyboard simulation"""
    print("\n🔄 CONTINUOUS SIMULATION TEST")
    print("=" * 40)
    print("📋 This runs forever (like the VLM system)")
    print("📝 Make sure text editor is active!")
    print("⏹️ Press Ctrl+C to stop")
    
    decisions = ['FORWARD', 'LEFT', 'RIGHT', 'STOP']
    
    try:
        count = 0
        while True:
            decision = decisions[count % 4]
            send_vlm_decision(decision)
            
            count += 1
            time.sleep(1.5)  # VLM response time simulation
            
    except KeyboardInterrupt:
        print("\n⏹️ Stopped by user")
        print("✅ Continuous simulation completed!")

def main():
    print("⌨️ VLM KEYBOARD SIMULATION TESTER")
    print("=" * 50)
    print("🎯 This proves that keyboard simulation works")
    print("📋 Choose test mode:")
    print("1. Immediate test (12 keystrokes)")
    print("2. Continuous simulation (until Ctrl+C)")
    print("3. Open Notepad only")
    print("4. Exit")
    
    while True:
        try:
            choice = input("\nEnter choice (1-4): ").strip()
            
            if choice == "1":
                test_immediate_keyboard()
                break
            elif choice == "2":
                print("📝 Make sure Notepad (or text editor) is open and active!")
                input("Press ENTER when ready...")
                test_continuous_simulation()
                break
            elif choice == "3":
                open_notepad()
                print("✅ Notepad opened! You can now run the VLM system.")
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
