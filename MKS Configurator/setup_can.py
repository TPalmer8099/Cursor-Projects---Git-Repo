#!/usr/bin/env python3
"""
Setup script for configuring CAN interface for MKS Configurator
"""

import os
import sys
import subprocess
import platform
import time

def is_admin():
    """Check if the script is running with administrator privileges"""
    try:
        if platform.system() == 'Windows':
            import ctypes
            return ctypes.windll.shell32.IsUserAnAdmin() != 0
        else:
            return os.geteuid() == 0
    except:
        return False

def setup_linux_can(interface='can0', bitrate=500000):
    """Setup CAN interface on Linux"""
    print(f"Setting up {interface} with bitrate {bitrate}...")
    
    # Check if can-utils is installed
    try:
        subprocess.run(['which', 'ip'], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        subprocess.run(['which', 'candump'], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except subprocess.CalledProcessError:
        print("Error: can-utils not found. Please install can-utils:")
        print("  sudo apt-get install can-utils")
        return False
    
    # Bring down interface if it's already up
    try:
        subprocess.run(['ip', 'link', 'set', 'down', interface], check=False)
    except:
        pass
    
    # Set up the CAN interface
    try:
        subprocess.run(['ip', 'link', 'set', interface, 'type', 'can', 'bitrate', str(bitrate)], check=True)
        subprocess.run(['ip', 'link', 'set', 'up', interface], check=True)
        print(f"Successfully set up {interface} with bitrate {bitrate}")
        
        # Test the interface
        print("Testing CAN interface (press Ctrl+C to stop)...")
        print("If you see no errors, the interface is working correctly.")
        subprocess.run(['candump', interface], check=False)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error setting up CAN interface: {e}")
        return False

def setup_windows_can():
    """Provide instructions for setting up CAN on Windows"""
    print("Windows CAN Setup Instructions:")
    print("1. Install the appropriate drivers for your CANable V2.0 device")
    print("2. For CANable V2.0, you'll typically need:")
    print("   - Canable drivers: https://canable.io/getting-started.html")
    print("   - PCAN-View software: https://www.peak-system.com/PCAN-View.242.0.html")
    print("\nAlternatively, you can use python-can's socketcan interface with:")
    print("  - Virtual CAN (vcan): Not suitable for real hardware")
    print("  - PCAN-Basic API: Requires PCAN hardware")
    print("\nFor more information, visit: https://python-can.readthedocs.io/en/master/interfaces/socketcan.html")
    return True

def setup_macos_can():
    """Provide instructions for setting up CAN on macOS"""
    print("macOS CAN Setup Instructions:")
    print("1. For CANable V2.0, install the appropriate drivers")
    print("2. You can use SocketCAN via a virtual machine running Linux")
    print("3. Alternatively, use a CAN-to-USB adapter with macOS drivers")
    print("\nFor more information, visit: https://python-can.readthedocs.io/en/master/interfaces.html")
    return True

def main():
    """Main function"""
    print("MKS Configurator - CAN Interface Setup")
    print("======================================")
    
    # Check for admin privileges
    if not is_admin():
        print("Error: This script requires administrator/root privileges.")
        print("Please run as administrator (Windows) or with sudo (Linux/macOS).")
        return
    
    # Detect operating system
    system = platform.system()
    print(f"Detected operating system: {system}")
    
    if system == 'Linux':
        # Get interface and bitrate
        interface = input("Enter CAN interface name [can0]: ").strip() or 'can0'
        bitrate_str = input("Enter bitrate [500000]: ").strip() or '500000'
        try:
            bitrate = int(bitrate_str)
        except ValueError:
            print("Error: Bitrate must be a number")
            return
        
        setup_linux_can(interface, bitrate)
    
    elif system == 'Windows':
        setup_windows_can()
    
    elif system == 'Darwin':  # macOS
        setup_macos_can()
    
    else:
        print(f"Unsupported operating system: {system}")
        return
    
    print("\nSetup complete. You can now run the MKS Configurator.")

if __name__ == "__main__":
    main()
