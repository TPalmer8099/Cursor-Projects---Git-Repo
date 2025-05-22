# MKS Configurator

A modern GUI tool for configuring and controlling MKS SERVO42D/57D_CAN stepper motors using a CANable V2.0 interface.

![MKS Configurator Screenshot](screenshot.png)

## Features

- Connect to MKS SERVO42D/57D_CAN motors via CANable V2.0
- Configure motor parameters:
  - Work mode (CR_OPEN, CR_CLOSE, CR_vFOC, SR_OPEN, SR_CLOSE, SR_vFOC)
  - Current settings (working current and holding current)
  - Subdivision settings
  - Direction and enable pin configuration
  - Protection settings
- Control motors:
  - Speed mode control
  - Position mode control (relative/absolute by pulses/axis)
  - Homing and limit switch configuration
  - Emergency stop
- Real-time monitoring:
  - Encoder value and angle
  - Motor speed
  - IO status
  - Error monitoring

## Requirements

- Python 3.6+
- PyQt5
- python-can
- CANable V2.0 hardware
- MKS SERVO42D/57D_CAN stepper motors

## Installation

1. Clone this repository:
   ```
   git clone https://github.com/yourusername/mks-configurator.git
   cd mks-configurator
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Set up the CANable V2.0 interface:
   - Connect the CANable V2.0 to your computer via USB
   - Install the necessary drivers for your operating system
   - Set up the CAN interface (instructions vary by OS)

## Usage

1. Connect your CANable V2.0 to your computer and to the MKS SERVO42D/57D_CAN motors.

2. Run the MKS Configurator:
   ```
   python mks_configurator.py
   ```

3. In the Connection tab:
   - Select the appropriate CAN channel (e.g., can0, slcan0)
   - Set the bitrate (default is 500K)
   - Click "Connect"
   - Set the motor ID (default is 1)

4. Use the Configuration tab to set motor parameters:
   - Work mode
   - Current settings
   - Motion settings
   - Protection settings

5. Use the Control tab to control the motor:
   - Speed control: Set direction, speed, and acceleration
   - Position control: Set mode, direction, position, speed, and acceleration
   - Homing: Configure and execute homing operations
   - Emergency stop: Immediately stop the motor in case of emergency

6. Use the Monitoring tab to view real-time motor data:
   - Position information (encoder value, angle, pulses)
   - Speed information
   - Error information
   - IO status
   - Motor status

## CAN Interface Setup

### Linux

1. Install can-utils:
   ```
   sudo apt-get install can-utils
   ```

2. Set up the CAN interface:
   ```
   sudo ip link set can0 type can bitrate 500000
   sudo ip link set up can0
   ```

### Windows

1. Install appropriate drivers for CANable V2.0
2. Use software like PCAN-View or similar to set up the CAN interface

## Troubleshooting

- **Connection Issues**: Ensure the CANable V2.0 is properly connected and the correct channel is selected.
- **Communication Errors**: Verify that the bitrate matches the motor's configured bitrate (default is 500K).
- **Motor Not Responding**: Check the motor ID setting and ensure it matches the ID set on the motor.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Based on the MKS SERVO42D/57D_CAN User Manual V1.0.4
- Uses python-can for CAN communication
- Uses PyQt5 for the graphical user interface
