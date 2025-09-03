#!/usr/bin/env python3
"""
Simple script to get local IP address
Useful for Duckiebot VLM setup
"""

import socket
import subprocess
import platform

def get_local_ip_socket():
    """Get local IP using socket method"""
    try:
        # Connect to a remote address (doesn't actually send data)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        return local_ip
    except Exception as e:
        return f"Error: {e}"

def get_local_ip_windows():
    """Get local IP on Windows using ipconfig"""
    try:
        result = subprocess.run(['ipconfig'], capture_output=True, text=True)
        lines = result.stdout.split('\n')
        
        current_adapter = ""
        for line in lines:
            if "adapter" in line.lower() and ("wifi" in line.lower() or "wireless" in line.lower()):
                current_adapter = line.strip()
            elif "IPv4 Address" in line and "wifi" in current_adapter.lower():
                ip = line.split(':')[1].strip().split('(')[0]
                return ip
    except Exception as e:
        return f"Error: {e}"
    return None

def main():
    print("🌐 Local IP Address Detection")
    print("="*40)
    
    # Method 1: Socket method (most reliable)
    ip_socket = get_local_ip_socket()
    print(f"Socket method: {ip_socket}")
    
    # Method 2: Platform-specific method
    if platform.system() == "Windows":
        ip_windows = get_local_ip_windows()
        if ip_windows:
            print(f"Windows method: {ip_windows}")
    
    print(f"\n🤖 For Duckiebot VLM access:")
    print(f"   - VLM Dashboard: http://{ip_socket}:5000")
    print(f"   - Screen Capture VLM: http://{ip_socket}:3000")
    print(f"   - llama.cpp server: http://{ip_socket}:8080")

if __name__ == "__main__":
    main()
