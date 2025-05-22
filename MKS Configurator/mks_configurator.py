import sys
import time
import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                            QLabel, QPushButton, QComboBox, QLineEdit, QTabWidget, 
                            QGroupBox, QGridLayout, QSpinBox, QDoubleSpinBox, QCheckBox,
                            QSlider, QProgressBar, QMessageBox, QFrame, QSplitter)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot, QThread
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon
import can

class CANInterface:
    """Interface for communicating with MKS motors via CAN bus"""
    
    def __init__(self):
        self.bus = None
        self.connected = False
        self.motor_id = 1  # Default motor ID
        
    def connect(self, channel='can0', bitrate=500000):
        """Connect to the CAN bus"""
        try:
            self.bus = can.interface.Bus(channel=channel, bustype='socketcan', bitrate=bitrate)
            self.connected = True
            return True
        except Exception as e:
            print(f"Error connecting to CAN bus: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from the CAN bus"""
        if self.bus:
            self.bus.shutdown()
            self.connected = False
    
    def calculate_crc(self, data):
        """Calculate CRC for MKS CAN messages"""
        crc = 0
        for byte in data:
            crc = (crc + byte) & 0xFF
        return crc
    
    def send_command(self, command, data=None):
        """Send a command to the motor"""
        if not self.connected or not self.bus:
            return False, "Not connected to CAN bus"
        
        # Prepare message data
        message_data = [self.motor_id]
        
        if data:
            message_data.extend(data)
        
        # Add command code
        message_data.append(command)
        
        # Add CRC
        crc = self.calculate_crc(message_data)
        message_data.append(crc)
        
        # Create and send CAN message
        try:
            msg = can.Message(
                arbitration_id=self.motor_id,
                data=bytes(message_data),
                is_extended_id=False
            )
            self.bus.send(msg)
            return True, "Command sent successfully"
        except Exception as e:
            return False, f"Error sending command: {e}"
    
    def receive_response(self, timeout=1.0):
        """Receive a response from the motor"""
        if not self.connected or not self.bus:
            return False, "Not connected to CAN bus", None
        
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg:
                # Verify CRC
                received_crc = msg.data[-1]
                calculated_crc = self.calculate_crc(msg.data[:-1])
                
                if received_crc == calculated_crc:
                    return True, "Response received", msg.data
                else:
                    return False, "CRC check failed", msg.data
            else:
                return False, "No response received", None
        except Exception as e:
            return False, f"Error receiving response: {e}", None
    
    def set_motor_id(self, motor_id):
        """Set the motor ID for communication"""
        self.motor_id = motor_id

class MKSCommands:
    """Implementation of MKS motor commands"""
    
    def __init__(self, can_interface):
        self.can = can_interface
    
    # Read parameter commands
    
    def read_encoder_value_carry(self):
        """Read the encoder value (carry) - Command 0x30"""
        success, message = self.can.send_command(0x30)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                # Parse the response according to the manual
                code = data[1]
                if code == 0x30:
                    # Extract carry (int32) and value (uint16)
                    carry = int.from_bytes(data[2:6], byteorder='little', signed=True)
                    value = int.from_bytes(data[6:8], byteorder='little', signed=False)
                    return True, {"carry": carry, "value": value}
            return False, message
        return False, message
    
    def read_encoder_value_addition(self):
        """Read the encoder value (addition) - Command 0x31"""
        success, message = self.can.send_command(0x31)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x31:
                    # Extract value (int48)
                    value = int.from_bytes(data[2:8], byteorder='little', signed=True)
                    return True, {"value": value}
            return False, message
        return False, message
    
    def read_motor_speed(self):
        """Read the real-time speed of the motor (RPM) - Command 0x32"""
        success, message = self.can.send_command(0x32)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x32:
                    # Extract speed (int16)
                    speed = int.from_bytes(data[2:4], byteorder='little', signed=True)
                    return True, {"speed": speed}
            return False, message
        return False, message
    
    def read_pulses_received(self):
        """Read the number of pulses received - Command 0x33"""
        success, message = self.can.send_command(0x33)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x33:
                    # Extract pulses (int32)
                    pulses = int.from_bytes(data[2:6], byteorder='little', signed=True)
                    return True, {"pulses": pulses}
            return False, message
        return False, message
    
    def read_io_status(self):
        """Read the IO Ports status - Command 0x34"""
        success, message = self.can.send_command(0x34)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x34:
                    # Extract status (uint8)
                    status = data[2]
                    # Parse individual bits
                    io_status = {
                        "IN_1": bool(status & 0x01),
                        "IN_2": bool(status & 0x02),
                        "OUT_1": bool(status & 0x04),
                        "OUT_2": bool(status & 0x08)
                    }
                    return True, io_status
            return False, message
        return False, message
    
    def read_angle_error(self):
        """Read the error of the motor shaft angle - Command 0x39"""
        success, message = self.can.send_command(0x39)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x39:
                    # Extract error (int32)
                    error = int.from_bytes(data[2:6], byteorder='little', signed=True)
                    # Convert to degrees (0~51200 corresponds to 0~360°)
                    error_degrees = (error * 360) / 51200
                    return True, {"error": error, "error_degrees": error_degrees}
            return False, message
        return False, message
    
    # Set parameter commands
    
    def calibrate_encoder(self):
        """Calibrate the encoder - Command 0x80"""
        success, message = self.can.send_command(0x80, [0x00])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x80:
                    status = data[2]
                    status_messages = {
                        0: "Calibrating...",
                        1: "Calibrated successfully",
                        2: "Calibration failed"
                    }
                    return True, {"status": status, "message": status_messages.get(status, "Unknown status")}
            return False, message
        return False, message
    
    def set_work_mode(self, mode):
        """Set the work mode - Command 0x82
        mode: 0=CR_OPEN, 1=CR_CLOSE, 2=CR_vFOC, 3=SR_OPEN, 4=SR_CLOSE, 5=SR_vFOC
        """
        if mode not in range(6):
            return False, "Invalid mode value"
        
        success, message = self.can.send_command(0x82, [mode])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x82:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_working_current(self, current_ma):
        """Set the working current in mA - Command 0x83"""
        # Convert current to bytes (uint16)
        current_bytes = current_ma.to_bytes(2, byteorder='little')
        
        success, message = self.can.send_command(0x83, list(current_bytes))
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x83:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_holding_current_percentage(self, percentage):
        """Set the holding current percentage - Command 0x9B
        percentage: 0=10%, 1=20%, ..., 8=90%
        """
        if percentage not in range(9):
            return False, "Invalid percentage value"
        
        success, message = self.can.send_command(0x9B, [percentage])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x9B:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_subdivision(self, microstep):
        """Set subdivision - Command 0x84"""
        success, message = self.can.send_command(0x84, [microstep])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x84:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_enable_pin(self, mode):
        """Set the active of the En pin - Command 0x85
        mode: 0=active low, 1=active high, 2=always enabled
        """
        if mode not in range(3):
            return False, "Invalid enable pin mode value"
        
        success, message = self.can.send_command(0x85, [mode])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x85:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_direction(self, direction):
        """Set the direction of motor rotation - Command 0x86
        direction: 0=CW, 1=CCW
        """
        if direction not in [0, 1]:
            return False, "Invalid direction value"
        
        success, message = self.can.send_command(0x86, [direction])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x86:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_auto_screen_off(self, enable):
        """Set auto turn off the screen function - Command 0x87
        enable: 0=disabled, 1=enabled
        """
        if enable not in [0, 1]:
            return False, "Invalid enable value"
        
        success, message = self.can.send_command(0x87, [enable])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x87:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_protection(self, enable):
        """Set the motor shaft locked-rotor protection function - Command 0x88
        enable: 0=disabled, 1=enabled
        """
        if enable not in [0, 1]:
            return False, "Invalid enable value"
        
        success, message = self.can.send_command(0x88, [enable])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x88:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_subdivision_interpolation(self, enable):
        """Set the subdivision interpolation function - Command 0x89
        enable: 0=disabled, 1=enabled
        """
        if enable not in [0, 1]:
            return False, "Invalid enable value"
        
        success, message = self.can.send_command(0x89, [enable])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x89:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_can_bitrate(self, bitrate):
        """Set the CAN bitRate - Command 0x8A
        bitrate: 0=125K, 1=250K, 2=500K, 3=1M
        """
        if bitrate not in range(4):
            return False, "Invalid bitrate value"
        
        success, message = self.can.send_command(0x8A, [bitrate])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x8A:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_can_id(self, can_id):
        """Set the CAN ID - Command 0x8B
        can_id: 0-2047 (0 is broadcast address)
        """
        if can_id < 0 or can_id > 2047:
            return False, "Invalid CAN ID value"
        
        # Convert CAN ID to bytes (uint16)
        can_id_bytes = can_id.to_bytes(2, byteorder='little')
        
        success, message = self.can.send_command(0x8B, list(can_id_bytes))
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x8B:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_slave_respond_active(self, respond, active):
        """Set the slave respond and active - Command 0x8C
        respond: 0=disabled, 1=enabled
        active: 0=disabled, 1=enabled
        """
        if respond not in [0, 1] or active not in [0, 1]:
            return False, "Invalid respond or active value"
        
        success, message = self.can.send_command(0x8C, [respond, active])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x8C:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_key_lock(self, lock):
        """Set the key lock or unlock - Command 0x8F
        lock: 0=unlock, 1=lock
        """
        if lock not in [0, 1]:
            return False, "Invalid lock value"
        
        success, message = self.can.send_command(0x8F, [lock])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x8F:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_group_id(self, group_id):
        """Set the group ID - Command 0x8D
        group_id: 1-2047
        """
        if group_id < 1 or group_id > 2047:
            return False, "Invalid group ID value"
        
        # Convert group ID to bytes (uint16)
        group_id_bytes = group_id.to_bytes(2, byteorder='little')
        
        success, message = self.can.send_command(0x8D, list(group_id_bytes))
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x8D:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_home_parameters(self, home_trig, home_dir, home_speed, end_limit):
        """Set the parameter of home - Command 0x90
        home_trig: 0=Low, 1=High
        home_dir: 0=CW, 1=CCW
        home_speed: 0-3000 RPM
        end_limit: 0=disabled, 1=enabled
        """
        if home_trig not in [0, 1] or home_dir not in [0, 1] or end_limit not in [0, 1]:
            return False, "Invalid parameter value"
        if home_speed < 0 or home_speed > 3000:
            return False, "Invalid home speed value"
        
        # Convert home speed to bytes (uint16)
        home_speed_bytes = home_speed.to_bytes(2, byteorder='little')
        
        success, message = self.can.send_command(0x90, [home_trig, home_dir, home_speed_bytes[0], home_speed_bytes[1], end_limit])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x90:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def go_home(self):
        """Go home - Command 0x91"""
        success, message = self.can.send_command(0x91)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x91:
                    status = data[2]
                    status_messages = {
                        0: "Go home failed",
                        1: "Go home started",
                        2: "Go home successful"
                    }
                    return True, {"status": status, "message": status_messages.get(status, "Unknown status")}
            return False, message
        return False, message
    
    def set_current_position_to_zero(self):
        """Set Current Axis to zero - Command 0x92"""
        success, message = self.can.send_command(0x92)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x92:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_zero_mode_parameters(self, mode, enable, speed, direction):
        """Set the parameter of 0_Mode - Command 0x9A
        mode: 0=Disable, 1=DirMode, 2=NearMode
        enable: 0=clean zero, 1=set zero
        speed: 0-4 (0=slowest, 4=fastest)
        direction: 0=CW, 1=CCW
        """
        if mode not in range(3) or enable not in [0, 1] or speed not in range(5) or direction not in [0, 1]:
            return False, "Invalid parameter value"
        
        success, message = self.can.send_command(0x9A, [mode, enable, speed, direction])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x9A:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def set_limit_port_remap(self, enable):
        """Set limit port remap - Command 0x9E
        enable: 0=disable remap limit port, 1=enable remap limit port
        """
        if enable not in [0, 1]:
            return False, "Invalid enable value"
        
        success, message = self.can.send_command(0x9E, [enable])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x9E:
                    status = data[2]
                    return True, {"status": status, "message": "Set successfully" if status == 1 else "Set failed"}
            return False, message
        return False, message
    
    def restore_default_parameters(self):
        """Restore the default parameter - Command 0x3F"""
        success, message = self.can.send_command(0x3F)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0x3F:
                    status = data[2]
                    return True, {"status": status, "message": "Restore successful" if status == 1 else "Restore failed"}
            return False, message
        return False, message
    
    # Motor control commands
    
    def run_motor_speed_mode(self, direction, speed, acceleration):
        """Run the motor in speed mode - Command 0xF6
        direction: 0=CCW, 1=CW
        speed: 0-3000 RPM
        acceleration: 0-255
        """
        if direction not in [0, 1]:
            return False, "Invalid direction value"
        if speed < 0 or speed > 3000:
            return False, "Invalid speed value"
        if acceleration < 0 or acceleration > 255:
            return False, "Invalid acceleration value"
        
        # Prepare the command data according to the manual
        # byte2: The highest bit indicates the direction, the lower 4 bits and byte3 together indicate the speed
        # byte3: The lower 4 bits of byte2 and byte3 together indicate speed
        
        # Set direction in highest bit of byte2
        byte2 = (direction << 7) | ((speed >> 8) & 0x0F)
        # Set lower 8 bits of speed in byte3
        byte3 = speed & 0xFF
        
        success, message = self.can.send_command(0xF6, [byte2, byte3, acceleration])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0xF6:
                    status = data[2]
                    return True, {"status": status, "message": "Run successfully" if status == 1 else "Run failed"}
            return False, message
        return False, message
    
    def stop_motor_speed_mode(self, acceleration=0):
        """Stop the motor in speed mode - Command 0xF6
        acceleration: 0=immediate stop, 1-255=deceleration stop
        """
        if acceleration < 0 or acceleration > 255:
            return False, "Invalid acceleration value"
        
        # For stop command, direction and speed are set to 0
        success, message = self.can.send_command(0xF6, [0, 0, acceleration])
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0xF6:
                    status = data[2]
                    status_messages = {
                        0: "Stop failed",
                        1: "Starting to stop",
                        2: "Stop completed"
                    }
                    return True, {"status": status, "message": status_messages.get(status, "Unknown status")}
            return False, message
        return False, message
    
    def emergency_stop(self):
        """Emergency stop the motor - Command 0xF7"""
        success, message = self.can.send_command(0xF7)
        if success:
            success, message, data = self.can.receive_response()
            if success and data:
                code = data[1]
                if code == 0xF7:
                    status = data[2]
                    return True, {"status": status, "message": "Stop successful" if status == 1 else "Stop failed"}
            return False, message
        return False, message

class DataMonitorThread(QThread):
    """Thread for continuously monitoring motor data"""
    
    # Define signals for data updates
    encoder_data_updated = pyqtSignal(dict)
    speed_data_updated = pyqtSignal(dict)
    io_data_updated = pyqtSignal(dict)
    error_data_updated = pyqtSignal(dict)
    
    def __init__(self, mks_commands):
        super().__init__()
        self.mks_commands = mks_commands
        self.running = False
    
    def run(self):
        self.running = True
        while self.running:
            # Read encoder value
            success, data = self.mks_commands.read_encoder_value_addition()
            if success:
                self.encoder_data_updated.emit(data)
            
            # Read motor speed
            success, data = self.mks_commands.read_motor_speed()
            if success:
                self.speed_data_updated.emit(data)
            
            # Read IO status
            success, data = self.mks_commands.read_io_status()
            if success:
                self.io_data_updated.emit(data)
            
            # Read angle error
            success, data = self.mks_commands.read_angle_error()
            if success:
                self.error_data_updated.emit(data)
            
            # Sleep to avoid overwhelming the CAN bus
            time.sleep(0.2)
    
    def stop(self):
        self.running = False
        self.wait()

class MKSConfiguratorApp(QMainWindow):
    """Main application window for MKS Configurator"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize CAN interface and commands
        self.can_interface = CANInterface()
        self.mks_commands = MKSCommands(self.can_interface)
        
        # Initialize UI
        self.init_ui()
        
        # Initialize data monitor thread
        self.data_monitor = None
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("MKS Motor Configurator")
        self.setGeometry(100, 100, 1200, 800)
        
        # Set application style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QTabWidget::pane {
                border: 1px solid #cccccc;
                background-color: white;
                border-radius: 4px;
            }
            QTabBar::tab {
                background-color: #e0e0e0;
                border: 1px solid #cccccc;
                border-bottom-color: #cccccc;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                min-width: 8ex;
                padding: 8px 16px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: white;
                border-bottom-color: white;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #cccccc;
                border-radius: 4px;
                margin-top: 16px;
                padding-top: 16px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #4a86e8;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background-color: #3a76d8;
            }
            QPushButton:pressed {
                background-color: #2a66c8;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
                border: 1px solid #cccccc;
                border-radius: 4px;
                padding: 4px;
            }
            QProgressBar {
                border: 1px solid #cccccc;
                border-radius: 4px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #4a86e8;
                width: 10px;
                margin: 0.5px;
            }
        """)
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create tab widget
        tab_widget = QTabWidget()
        main_layout.addWidget(tab_widget)
        
        # Create tabs
        connection_tab = QWidget()
        configuration_tab = QWidget()
        control_tab = QWidget()
        monitoring_tab = QWidget()
        
        tab_widget.addTab(connection_tab, "Connection")
        tab_widget.addTab(configuration_tab, "Configuration")
        tab_widget.addTab(control_tab, "Control")
        tab_widget.addTab(monitoring_tab, "Monitoring")
        
        # Setup each tab
        self.setup_connection_tab(connection_tab)
        self.setup_configuration_tab(configuration_tab)
        self.setup_control_tab(control_tab)
        self.setup_monitoring_tab(monitoring_tab)
        
        # Status bar
        self.statusBar().showMessage("Ready")
    
    def setup_connection_tab(self, tab):
        """Setup the connection tab"""
        layout = QVBoxLayout(tab)
        
        # Connection group
        connection_group = QGroupBox("CAN Connection")
        connection_layout = QGridLayout(connection_group)
        
        # Channel selection
        channel_label = QLabel("Channel:")
        self.channel_combo = QComboBox()
        self.channel_combo.addItems(["can0", "can1", "slcan0", "vcan0"])
        connection_layout.addWidget(channel_label, 0, 0)
        connection_layout.addWidget(self.channel_combo, 0, 1)
        
        # Bitrate selection
        bitrate_label = QLabel("Bitrate:")
        self.bitrate_combo = QComboBox()
        self.bitrate_combo.addItems(["125000", "250000", "500000", "1000000"])
        self.bitrate_combo.setCurrentText("500000")  # Default to 500K
        connection_layout.addWidget(bitrate_label, 1, 0)
        connection_layout.addWidget(self.bitrate_combo, 1, 1)
        
        # Connect button
        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_to_can)
        connection_layout.addWidget(self.connect_button, 2, 0, 1, 2)
        
        layout.addWidget(connection_group)
        
        # Motor ID group
        motor_id_group = QGroupBox("Motor ID")
        motor_id_layout = QGridLayout(motor_id_group)
        
        # Motor ID selection
        motor_id_label = QLabel("Motor ID:")
        self.motor_id_spin = QSpinBox()
        self.motor_id_spin.setRange(1, 127)
        self.motor_id_spin.setValue(1)
        self.motor_id_spin.valueChanged.connect(self.set_motor_id)
        motor_id_layout.addWidget(motor_id_label, 0, 0)
        motor_id_layout.addWidget(self.motor_id_spin, 0, 1)
        
        # Group ID
        group_id_label = QLabel("Group ID:")
        self.group_id_spin = QSpinBox()
        self.group_id_spin.setRange(1, 2047)
        self.group_id_spin.setValue(80)  # Default to 0x50
        motor_id_layout.addWidget(group_id_label, 1, 0)
        motor_id_layout.addWidget(self.group_id_spin, 1, 1)
        
        # Set Group ID button
        self.set_group_id_button = QPushButton("Set Group ID")
        self.set_group_id_button.setEnabled(False)
        self.set_group_id_button.clicked.connect(self.set_group_id)
        motor_id_layout.addWidget(self.set_group_id_button, 2, 0, 1, 2)
        
        layout.addWidget(motor_id_group)
        
        # CAN Rate group
        can_rate_group = QGroupBox("CAN Rate")
        can_rate_layout = QGridLayout(can_rate_group)
        
        # CAN Rate selection
        can_rate_label = QLabel("CAN Rate:")
        self.can_rate_combo = QComboBox()
        self.can_rate_combo.addItems(["125K", "250K", "500K", "1M"])
        self.can_rate_combo.setCurrentText("500K")  # Default to 500K
        can_rate_layout.addWidget(can_rate_label, 0, 0)
        can_rate_layout.addWidget(self.can_rate_combo, 0, 1)
        
        # Set CAN Rate button
        self.set_can_rate_button = QPushButton("Set CAN Rate")
        self.set_can_rate_button.setEnabled(False)
        self.set_can_rate_button.clicked.connect(self.set_can_rate)
        can_rate_layout.addWidget(self.set_can_rate_button, 1, 0, 1, 2)
        
        layout.addWidget(can_rate_group)
        
        # Add stretch to push everything to the top
        layout.addStretch()
    
    def setup_configuration_tab(self, tab):
        """Setup the configuration tab"""
        layout = QVBoxLayout(tab)
        
        # Work Mode group
        work_mode_group = QGroupBox("Work Mode")
        work_mode_layout = QGridLayout(work_mode_group)
        
        # Work Mode selection
        work_mode_label = QLabel("Work Mode:")
        self.work_mode_combo = QComboBox()
        self.work_mode_combo.addItems([
            "CR_OPEN - Pulse Open Loop",
            "CR_CLOSE - Pulse Closed Loop",
            "CR_vFOC - Pulse FOC",
            "SR_OPEN - Serial Open Loop",
            "SR_CLOSE - Serial Closed Loop",
            "SR_vFOC - Serial FOC"
        ])
        self.work_mode_combo.setCurrentIndex(5)  # Default to SR_vFOC
        work_mode_layout.addWidget(work_mode_label, 0, 0)
        work_mode_layout.addWidget(self.work_mode_combo, 0, 1)
        
        # Set Work Mode button
        self.set_work_mode_button = QPushButton("Set Work Mode")
        self.set_work_mode_button.setEnabled(False)
        self.set_work_mode_button.clicked.connect(self.set_work_mode)
        work_mode_layout.addWidget(self.set_work_mode_button, 1, 0, 1, 2)
        
        layout.addWidget(work_mode_group)
        
        # Current Settings group
        current_group = QGroupBox("Current Settings")
        current_layout = QGridLayout(current_group)
        
        # Working Current
        working_current_label = QLabel("Working Current (mA):")
        self.working_current_spin = QSpinBox()
        self.working_current_spin.setRange(0, 5200)
        self.working_current_spin.setSingleStep(200)
        self.working_current_spin.setValue(1600)  # Default for 42D
        current_layout.addWidget(working_current_label, 0, 0)
        current_layout.addWidget(self.working_current_spin, 0, 1)
        
        # Holding Current Percentage
        holding_current_label = QLabel("Holding Current (%):")
        self.holding_current_combo = QComboBox()
        self.holding_current_combo.addItems(["10%", "20%", "30%", "40%", "50%", "60%", "70%", "80%", "90%"])
        self.holding_current_combo.setCurrentText("50%")  # Default
        current_layout.addWidget(holding_current_label, 1, 0)
        current_layout.addWidget(self.holding_current_combo, 1, 1)
        
        # Set Current buttons
        self.set_working_current_button = QPushButton("Set Working Current")
        self.set_working_current_button.setEnabled(False)
        self.set_working_current_button.clicked.connect(self.set_working_current)
        current_layout.addWidget(self.set_working_current_button, 2, 0)
        
        self.set_holding_current_button = QPushButton("Set Holding Current")
        self.set_holding_current_button.setEnabled(False)
        self.set_holding_current_button.clicked.connect(self.set_holding_current)
        current_layout.addWidget(self.set_holding_current_button, 2, 1)
        
        layout.addWidget(current_group)
        
        # Motion Settings group
        motion_group = QGroupBox("Motion Settings")
        motion_layout = QGridLayout(motion_group)
        
        # Subdivision
        subdivision_label = QLabel("Subdivision:")
        self.subdivision_combo = QComboBox()
        self.subdivision_combo.addItems(["1", "2", "4", "8", "16", "32", "64", "128", "256"])
        self.subdivision_combo.setCurrentText("16")  # Default
        motion_layout.addWidget(subdivision_label, 0, 0)
        motion_layout.addWidget(self.subdivision_combo, 0, 1)
        
        # Direction
        direction_label = QLabel("Direction:")
        self.direction_combo = QComboBox()
        self.direction_combo.addItems(["CW", "CCW"])
        motion_layout.addWidget(direction_label, 1, 0)
        motion_layout.addWidget(self.direction_combo, 1, 1)
        
        # Enable Pin
        enable_pin_label = QLabel("Enable Pin:")
        self.enable_pin_combo = QComboBox()
        self.enable_pin_combo.addItems(["Low Active", "High Active", "Always Enabled"])
        motion_layout.addWidget(enable_pin_label, 2, 0)
        motion_layout.addWidget(self.enable_pin_combo, 2, 1)
        
        # Set Motion Settings buttons
        self.set_subdivision_button = QPushButton("Set Subdivision")
        self.set_subdivision_button.setEnabled(False)
        self.set_subdivision_button.clicked.connect(self.set_subdivision)
        motion_layout.addWidget(self.set_subdivision_button, 3, 0)
        
        self.set_direction_button = QPushButton("Set Direction")
        self.set_direction_button.setEnabled(False)
        self.set_direction_button.clicked.connect(self.set_direction)
        motion_layout.addWidget(self.set_direction_button, 3, 1)
        
        self.set_enable_pin_button = QPushButton("Set Enable Pin")
        self.set_enable_pin_button.setEnabled(False)
        self.set_enable_pin_button.clicked.connect(self.set_enable_pin)
        motion_layout.addWidget(self.set_enable_pin_button, 4, 0, 1, 2)
        
        layout.addWidget(motion_group)
        
        # Protection Settings group
        protection_group = QGroupBox("Protection Settings")
        protection_layout = QGridLayout(protection_group)
        
        # Locked-rotor Protection
        protection_label = QLabel("Locked-rotor Protection:")
        self.protection_combo = QComboBox()
        self.protection_combo.addItems(["Disabled", "Enabled"])
        protection_layout.addWidget(protection_label, 0, 0)
        protection_layout.addWidget(self.protection_combo, 0, 1)
        
        # Set Protection button
        self.set_protection_button = QPushButton("Set Protection")
        self.set_protection_button.setEnabled(False)
        self.set_protection_button.clicked.connect(self.set_protection)
        protection_layout.addWidget(self.set_protection_button, 1, 0, 1, 2)
        
        layout.addWidget(protection_group)
        
        # Limit Port Remap group
        limit_remap_group = QGroupBox("Limit Port Remap")
        limit_remap_layout = QGridLayout(limit_remap_group)
        
        # Limit Port Remap
        limit_remap_label = QLabel("Limit Port Remap:")
        self.limit_remap_combo = QComboBox()
        self.limit_remap_combo.addItems(["Disabled", "Enabled"])
        limit_remap_layout.addWidget(limit_remap_label, 0, 0)
        limit_remap_layout.addWidget(self.limit_remap_combo, 0, 1)
        
        # Set Limit Port Remap button
        self.set_limit_remap_button = QPushButton("Set Limit Port Remap")
        self.set_limit_remap_button.setEnabled(False)
        self.set_limit_remap_button.clicked.connect(self.set_limit_port_remap)
        limit_remap_layout.addWidget(self.set_limit_remap_button, 1, 0, 1, 2)
        
        # Note about Limit Port Remap
        limit_remap_note = QLabel("Note: Only for serial control mode.\nLeft limit -> En port\nRight limit -> Dir port")
        limit_remap_note.setStyleSheet("font-style: italic;")
        limit_remap_layout.addWidget(limit_remap_note, 2, 0, 1, 2)
        
        layout.addWidget(limit_remap_group)
        
        # Add stretch to push everything to the top
        layout.addStretch()
    
    def setup_control_tab(self, tab):
        """Setup the control tab"""
        layout = QVBoxLayout(tab)
        
        # Speed Control group
        speed_control_group = QGroupBox("Speed Control")
        speed_layout = QGridLayout(speed_control_group)
        
        # Direction
        direction_label = QLabel("Direction:")
        self.speed_direction_combo = QComboBox()
        self.speed_direction_combo.addItems(["CCW", "CW"])
        speed_layout.addWidget(direction_label, 0, 0)
        speed_layout.addWidget(self.speed_direction_combo, 0, 1)
        
        # Speed
        speed_label = QLabel("Speed (RPM):")
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(0, 3000)
        self.speed_spin.setValue(600)
        speed_layout.addWidget(speed_label, 1, 0)
        speed_layout.addWidget(self.speed_spin, 1, 1)
        
        # Acceleration
        acceleration_label = QLabel("Acceleration:")
        self.acceleration_spin = QSpinBox()
        self.acceleration_spin.setRange(0, 255)
        self.acceleration_spin.setValue(10)
        speed_layout.addWidget(acceleration_label, 2, 0)
        speed_layout.addWidget(self.acceleration_spin, 2, 1)
        
        # Run/Stop buttons
        self.run_speed_button = QPushButton("Run")
        self.run_speed_button.setEnabled(False)
        self.run_speed_button.clicked.connect(self.run_motor_speed)
        speed_layout.addWidget(self.run_speed_button, 3, 0)
        
        self.stop_speed_button = QPushButton("Stop")
        self.stop_speed_button.setEnabled(False)
        self.stop_speed_button.clicked.connect(self.stop_motor_speed)
        speed_layout.addWidget(self.stop_speed_button, 3, 1)
        
        layout.addWidget(speed_control_group)
        
        # Position Control group
        position_control_group = QGroupBox("Position Control")
        position_layout = QGridLayout(position_control_group)
        
        # Control Mode
        control_mode_label = QLabel("Control Mode:")
        self.position_mode_combo = QComboBox()
        self.position_mode_combo.addItems([
            "Relative by Pulses",
            "Absolute by Pulses",
            "Relative by Axis",
            "Absolute by Axis"
        ])
        position_layout.addWidget(control_mode_label, 0, 0)
        position_layout.addWidget(self.position_mode_combo, 0, 1)
        
        # Direction
        position_direction_label = QLabel("Direction:")
        self.position_direction_combo = QComboBox()
        self.position_direction_combo.addItems(["CCW", "CW"])
        position_layout.addWidget(position_direction_label, 1, 0)
        position_layout.addWidget(self.position_direction_combo, 1, 1)
        
        # Position/Pulses
        position_label = QLabel("Position/Pulses:")
        self.position_spin = QSpinBox()
        self.position_spin.setRange(-8388607, 8388607)
        self.position_spin.setValue(1600)  # 1 rotation at 16 subdivisions
        position_layout.addWidget(position_label, 2, 0)
        position_layout.addWidget(self.position_spin, 2, 1)
        
        # Speed
        position_speed_label = QLabel("Speed (RPM):")
        self.position_speed_spin = QSpinBox()
        self.position_speed_spin.setRange(0, 3000)
        self.position_speed_spin.setValue(600)
        position_layout.addWidget(position_speed_label, 3, 0)
        position_layout.addWidget(self.position_speed_spin, 3, 1)
        
        # Acceleration
        position_acceleration_label = QLabel("Acceleration:")
        self.position_acceleration_spin = QSpinBox()
        self.position_acceleration_spin.setRange(0, 255)
        self.position_acceleration_spin.setValue(10)
        position_layout.addWidget(position_acceleration_label, 4, 0)
        position_layout.addWidget(self.position_acceleration_spin, 4, 1)
        
        # Run/Stop buttons
        self.run_position_button = QPushButton("Run")
        self.run_position_button.setEnabled(False)
        self.run_position_button.clicked.connect(self.run_motor_position)
        position_layout.addWidget(self.run_position_button, 5, 0)
        
        self.stop_position_button = QPushButton("Stop")
        self.stop_position_button.setEnabled(False)
        self.stop_position_button.clicked.connect(self.stop_motor_position)
        position_layout.addWidget(self.stop_position_button, 5, 1)
        
        layout.addWidget(position_control_group)
        
        # Homing group
        homing_group = QGroupBox("Homing")
        homing_layout = QGridLayout(homing_group)
        
        # Home Direction
        home_direction_label = QLabel("Home Direction:")
        self.home_direction_combo = QComboBox()
        self.home_direction_combo.addItems(["CW", "CCW"])
        homing_layout.addWidget(home_direction_label, 0, 0)
        homing_layout.addWidget(self.home_direction_combo, 0, 1)
        
        # Home Speed
        home_speed_label = QLabel("Home Speed (RPM):")
        self.home_speed_spin = QSpinBox()
        self.home_speed_spin.setRange(30, 3000)
        self.home_speed_spin.setValue(120)
        homing_layout.addWidget(home_speed_label, 1, 0)
        homing_layout.addWidget(self.home_speed_spin, 1, 1)
        
        # Home Trigger Level
        home_trigger_label = QLabel("Trigger Level:")
        self.home_trigger_combo = QComboBox()
        self.home_trigger_combo.addItems(["Low", "High"])
        homing_layout.addWidget(home_trigger_label, 2, 0)
        homing_layout.addWidget(self.home_trigger_combo, 2, 1)
        
        # End Limit
        end_limit_label = QLabel("End Limit:")
        self.end_limit_combo = QComboBox()
        self.end_limit_combo.addItems(["Disabled", "Enabled"])
        homing_layout.addWidget(end_limit_label, 3, 0)
        homing_layout.addWidget(self.end_limit_combo, 3, 1)
        
        # Set Home Parameters button
        self.set_home_params_button = QPushButton("Set Home Parameters")
        self.set_home_params_button.setEnabled(False)
        self.set_home_params_button.clicked.connect(self.set_home_parameters)
        homing_layout.addWidget(self.set_home_params_button, 4, 0)
        
        # Go Home button
        self.go_home_button = QPushButton("Go Home")
        self.go_home_button.setEnabled(False)
        self.go_home_button.clicked.connect(self.go_home)
        homing_layout.addWidget(self.go_home_button, 4, 1)
        
        # Set Zero button
        self.set_zero_button = QPushButton("Set Current Position to Zero")
        self.set_zero_button.setEnabled(False)
        self.set_zero_button.clicked.connect(self.set_current_position_to_zero)
        homing_layout.addWidget(self.set_zero_button, 5, 0, 1, 2)
        
        layout.addWidget(homing_group)
        
        # Emergency Stop button
        self.emergency_stop_button = QPushButton("EMERGENCY STOP")
        self.emergency_stop_button.setEnabled(False)
        self.emergency_stop_button.setStyleSheet("""
            QPushButton {
                background-color: #d9534f;
                color: white;
                font-weight: bold;
                font-size: 16px;
                padding: 12px;
            }
            QPushButton:hover {
                background-color: #c9302c;
            }
            QPushButton:pressed {
                background-color: #ac2925;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        self.emergency_stop_button.clicked.connect(self.emergency_stop)
        layout.addWidget(self.emergency_stop_button)
        
        # Add stretch to push everything to the top
        layout.addStretch()
    
    def setup_monitoring_tab(self, tab):
        """Setup the monitoring tab"""
        layout = QVBoxLayout(tab)
        
        # Position Monitoring group
        position_group = QGroupBox("Position Monitoring")
        position_layout = QGridLayout(position_group)
        
        # Encoder Value
        encoder_label = QLabel("Encoder Value:")
        self.encoder_value_label = QLabel("0")
        self.encoder_value_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        position_layout.addWidget(encoder_label, 0, 0)
        position_layout.addWidget(self.encoder_value_label, 0, 1)
        
        # Angle
        angle_label = QLabel("Angle (degrees):")
        self.angle_label = QLabel("0.0°")
        self.angle_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        position_layout.addWidget(angle_label, 1, 0)
        position_layout.addWidget(self.angle_label, 1, 1)
        
        # Pulses Received
        pulses_label = QLabel("Pulses Received:")
        self.pulses_label = QLabel("0")
        self.pulses_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        position_layout.addWidget(pulses_label, 2, 0)
        position_layout.addWidget(self.pulses_label, 2, 1)
        
        layout.addWidget(position_group)
        
        # Speed Monitoring group
        speed_group = QGroupBox("Speed Monitoring")
        speed_layout = QGridLayout(speed_group)
        
        # Current Speed
        current_speed_label = QLabel("Current Speed (RPM):")
        self.current_speed_label = QLabel("0")
        self.current_speed_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        speed_layout.addWidget(current_speed_label, 0, 0)
        speed_layout.addWidget(self.current_speed_label, 0, 1)
        
        # Speed Bar
        speed_bar_label = QLabel("Speed:")
        self.speed_bar = QProgressBar()
        self.speed_bar.setRange(-3000, 3000)
        self.speed_bar.setValue(0)
        self.speed_bar.setFormat("%v RPM")
        self.speed_bar.setAlignment(Qt.AlignCenter)
        speed_layout.addWidget(speed_bar_label, 1, 0)
        speed_layout.addWidget(self.speed_bar, 1, 1)
        
        layout.addWidget(speed_group)
        
        # Error Monitoring group
        error_group = QGroupBox("Error Monitoring")
        error_layout = QGridLayout(error_group)
        
        # Angle Error
        angle_error_label = QLabel("Angle Error:")
        self.angle_error_label = QLabel("0.0°")
        self.angle_error_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        error_layout.addWidget(angle_error_label, 0, 0)
        error_layout.addWidget(self.angle_error_label, 0, 1)
        
        layout.addWidget(error_group)
        
        # IO Status group
        io_group = QGroupBox("IO Status")
        io_layout = QGridLayout(io_group)
        
        # IN_1
        in1_label = QLabel("IN_1 (Home/Left Limit):")
        self.in1_indicator = QLabel("OFF")
        self.in1_indicator.setStyleSheet("color: red; font-weight: bold;")
        io_layout.addWidget(in1_label, 0, 0)
        io_layout.addWidget(self.in1_indicator, 0, 1)
        
        # IN_2
        in2_label = QLabel("IN_2 (Right Limit):")
        self.in2_indicator = QLabel("OFF")
        self.in2_indicator.setStyleSheet("color: red; font-weight: bold;")
        io_layout.addWidget(in2_label, 1, 0)
        io_layout.addWidget(self.in2_indicator, 1, 1)
        
        # OUT_1
        out1_label = QLabel("OUT_1 (Stall Indication):")
        self.out1_indicator = QLabel("OFF")
        self.out1_indicator.setStyleSheet("color: red; font-weight: bold;")
        io_layout.addWidget(out1_label, 2, 0)
        io_layout.addWidget(self.out1_indicator, 2, 1)
        
        # OUT_2
        out2_label = QLabel("OUT_2:")
        self.out2_indicator = QLabel("OFF")
        self.out2_indicator.setStyleSheet("color: red; font-weight: bold;")
        io_layout.addWidget(out2_label, 3, 0)
        io_layout.addWidget(self.out2_indicator, 3, 1)
        
        layout.addWidget(io_group)
        
        # Motor Status group
        motor_status_group = QGroupBox("Motor Status")
        motor_status_layout = QGridLayout(motor_status_group)
        
        # Motor Status
        motor_status_label = QLabel("Status:")
        self.motor_status_label = QLabel("Unknown")
        self.motor_status_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        motor_status_layout.addWidget(motor_status_label, 0, 0)
        motor_status_layout.addWidget(self.motor_status_label, 0, 1)
        
        # Protection Status
        protection_status_label = QLabel("Protection Status:")
        self.protection_status_label = QLabel("Unknown")
        self.protection_status_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        motor_status_layout.addWidget(protection_status_label, 1, 0)
        motor_status_layout.addWidget(self.protection_status_label, 1, 1)
        
        layout.addWidget(motor_status_group)
        
        # Add stretch to push everything to the top
        layout.addStretch()
    
    # Connection methods
    
    def connect_to_can(self):
        """Connect to the CAN bus"""
        channel = self.channel_combo.currentText()
        bitrate = int(self.bitrate_combo.currentText())
        
        if self.can_interface.connected:
            # Disconnect if already connected
            self.can_interface.disconnect()
            self.connect_button.setText("Connect")
            self.statusBar().showMessage("Disconnected from CAN bus")
            
            # Disable buttons
            self.set_group_id_button.setEnabled(False)
            self.set_can_rate_button.setEnabled(False)
            self.set_work_mode_button.setEnabled(False)
            self.set_working_current_button.setEnabled(False)
            self.set_holding_current_button.setEnabled(False)
            self.set_subdivision_button.setEnabled(False)
            self.set_direction_button.setEnabled(False)
            self.set_enable_pin_button.setEnabled(False)
            self.set_protection_button.setEnabled(False)
            self.run_speed_button.setEnabled(False)
            self.stop_speed_button.setEnabled(False)
            self.run_position_button.setEnabled(False)
            self.stop_position_button.setEnabled(False)
            self.set_home_params_button.setEnabled(False)
            self.go_home_button.setEnabled(False)
            self.set_zero_button.setEnabled(False)
            self.emergency_stop_button.setEnabled(False)
            
            # Stop data monitor thread
            if self.data_monitor:
                self.data_monitor.stop()
                self.data_monitor = None
        else:
            # Connect to CAN bus
            success = self.can_interface.connect(channel=channel, bitrate=bitrate)
            
            if success:
                self.connect_button.setText("Disconnect")
                self.statusBar().showMessage(f"Connected to CAN bus on {channel} at {bitrate} bps")
                
                # Enable buttons
                self.set_group_id_button.setEnabled(True)
                self.set_can_rate_button.setEnabled(True)
                self.set_work_mode_button.setEnabled(True)
                self.set_working_current_button.setEnabled(True)
                self.set_holding_current_button.setEnabled(True)
                self.set_subdivision_button.setEnabled(True)
                self.set_direction_button.setEnabled(True)
                self.set_enable_pin_button.setEnabled(True)
                self.set_protection_button.setEnabled(True)
                self.run_speed_button.setEnabled(True)
                self.stop_speed_button.setEnabled(True)
                self.run_position_button.setEnabled(True)
                self.stop_position_button.setEnabled(True)
                self.set_home_params_button.setEnabled(True)
                self.go_home_button.setEnabled(True)
                self.set_zero_button.setEnabled(True)
                self.emergency_stop_button.setEnabled(True)
                
                # Start data monitor thread
                self.data_monitor = DataMonitorThread(self.mks_commands)
                self.data_monitor.encoder_data_updated.connect(self.update_encoder_data)
                self.data_monitor.speed_data_updated.connect(self.update_speed_data)
                self.data_monitor.io_data_updated.connect(self.update_io_data)
                self.data_monitor.error_data_updated.connect(self.update_error_data)
                self.data_monitor.start()
            else:
                QMessageBox.critical(self, "Connection Error", "Failed to connect to CAN bus")
    
    def set_motor_id(self, motor_id):
        """Set the motor ID for communication"""
        self.can_interface.set_motor_id(motor_id)
    
    # Configuration methods
    
    def set_work_mode(self):
        """Set the work mode"""
        mode_index = self.work_mode_combo.currentIndex()
        
        success, result = self.mks_commands.set_work_mode(mode_index)
        
        if success:
            self.statusBar().showMessage(f"Work mode set to {self.work_mode_combo.currentText()}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set work mode: {result}")
    
    def set_working_current(self):
        """Set the working current"""
        current = self.working_current_spin.value()
        
        success, result = self.mks_commands.set_working_current(current)
        
        if success:
            self.statusBar().showMessage(f"Working current set to {current} mA")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set working current: {result}")
    
    def set_holding_current(self):
        """Set the holding current percentage"""
        percentage_index = self.holding_current_combo.currentIndex()
        
        success, result = self.mks_commands.set_holding_current_percentage(percentage_index)
        
        if success:
            self.statusBar().showMessage(f"Holding current set to {self.holding_current_combo.currentText()}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set holding current: {result}")
    
    def set_subdivision(self):
        """Set subdivision"""
        subdivision_text = self.subdivision_combo.currentText()
        subdivision_value = int(subdivision_text)
        
        success, result = self.mks_commands.set_subdivision(subdivision_value)
        
        if success:
            self.statusBar().showMessage(f"Subdivision set to {subdivision_text}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set subdivision: {result}")
    
    def set_direction(self):
        """Set direction"""
        direction_index = self.direction_combo.currentIndex()
        
        success, result = self.mks_commands.set_direction(direction_index)
        
        if success:
            self.statusBar().showMessage(f"Direction set to {self.direction_combo.currentText()}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set direction: {result}")
    
    def set_enable_pin(self):
        """Set enable pin mode"""
        enable_pin_index = self.enable_pin_combo.currentIndex()
        
        success, result = self.mks_commands.set_enable_pin(enable_pin_index)
        
        if success:
            self.statusBar().showMessage(f"Enable pin set to {self.enable_pin_combo.currentText()}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set enable pin: {result}")
    
    def set_protection(self):
        """Set protection"""
        protection_index = self.protection_combo.currentIndex()
        
        success, result = self.mks_commands.set_protection(protection_index)
        
        if success:
            self.statusBar().showMessage(f"Protection {self.protection_combo.currentText()}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set protection: {result}")
    
    def set_limit_port_remap(self):
        """Set limit port remap"""
        remap_index = self.limit_remap_combo.currentIndex()
        
        success, result = self.mks_commands.set_limit_port_remap(remap_index)
        
        if success:
            self.statusBar().showMessage(f"Limit port remap {self.limit_remap_combo.currentText()}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set limit port remap: {result}")
    
    def set_group_id(self):
        """Set group ID"""
        group_id = self.group_id_spin.value()
        
        success, result = self.mks_commands.set_group_id(group_id)
        
        if success:
            self.statusBar().showMessage(f"Group ID set to {group_id}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set group ID: {result}")
    
    def set_can_rate(self):
        """Set CAN rate"""
        can_rate_text = self.can_rate_combo.currentText()
        can_rate_index = {"125K": 0, "250K": 1, "500K": 2, "1M": 3}[can_rate_text]
        
        success, result = self.mks_commands.set_can_bitrate(can_rate_index)
        
        if success:
            self.statusBar().showMessage(f"CAN rate set to {can_rate_text}")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set CAN rate: {result}")
    
    def set_home_parameters(self):
        """Set home parameters"""
        home_trig = self.home_trigger_combo.currentIndex()
        home_dir = self.home_direction_combo.currentIndex()
        home_speed = self.home_speed_spin.value()
        end_limit = self.end_limit_combo.currentIndex()
        
        success, result = self.mks_commands.set_home_parameters(home_trig, home_dir, home_speed, end_limit)
        
        if success:
            self.statusBar().showMessage("Home parameters set successfully")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set home parameters: {result}")
    
    def go_home(self):
        """Go home"""
        success, result = self.mks_commands.go_home()
        
        if success:
            status = result.get("status", 0)
            message = result.get("message", "Unknown status")
            self.statusBar().showMessage(message)
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to go home: {result}")
    
    def set_current_position_to_zero(self):
        """Set current position to zero"""
        success, result = self.mks_commands.set_current_position_to_zero()
        
        if success:
            self.statusBar().showMessage("Current position set to zero")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to set position to zero: {result}")
    
    # Control methods
    
    def run_motor_speed(self):
        """Run the motor in speed mode"""
        direction = self.speed_direction_combo.currentIndex()
        speed = self.speed_spin.value()
        acceleration = self.acceleration_spin.value()
        
        success, result = self.mks_commands.run_motor_speed_mode(direction, speed, acceleration)
        
        if success:
            self.statusBar().showMessage(f"Running motor at {speed} RPM")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to run motor: {result}")
    
    def stop_motor_speed(self):
        """Stop the motor in speed mode"""
        acceleration = self.acceleration_spin.value()
        
        success, result = self.mks_commands.stop_motor_speed_mode(acceleration)
        
        if success:
            self.statusBar().showMessage("Stopping motor")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to stop motor: {result}")
    
    def emergency_stop(self):
        """Emergency stop the motor"""
        success, result = self.mks_commands.emergency_stop()
        
        if success:
            self.statusBar().showMessage("Emergency stop activated")
        else:
            QMessageBox.warning(self, "Command Failed", f"Failed to emergency stop: {result}")
    
    def run_motor_position(self):
        """Run the motor in position mode"""
        mode_index = self.position_mode_combo.currentIndex()
        direction = self.position_direction_combo.currentIndex()
        position = self.position_spin.value()
        speed = self.position_speed_spin.value()
        acceleration = self.position_acceleration_spin.value()
        
        # Prepare command based on position mode
        if mode_index == 0:  # Relative by Pulses
            # Convert position to bytes (uint24)
            position_bytes = position.to_bytes(3, byteorder='little', signed=False)
            
            # Prepare the command data according to the manual
            # byte2: The highest bit indicates the direction, the lower 4 bits and byte3 together indicate the speed
            # byte3: The lower 4 bits of byte2 and byte3 together indicate speed
            byte2 = (direction << 7) | ((speed >> 8) & 0x0F)
            byte3 = speed & 0xFF
            
            # Send command
            success, message = self.can_interface.send_command(0xFD, [byte2, byte3, acceleration, 
                                                                     position_bytes[0], position_bytes[1], position_bytes[2]])
            
            if success:
                success, message, data = self.can_interface.receive_response()
                if success and data:
                    code = data[1]
                    if code == 0xFD:
                        status = data[2]
                        status_messages = {
                            0: "Run failed",
                            1: "Run starting",
                            2: "Run completed",
                            3: "End limit stopped"
                        }
                        self.statusBar().showMessage(status_messages.get(status, "Unknown status"))
                        return
            
            QMessageBox.warning(self, "Command Failed", f"Failed to run motor: {message}")
            
        elif mode_index == 1:  # Absolute by Pulses
            # Convert position to bytes (int24)
            position_bytes = position.to_bytes(3, byteorder='little', signed=True)
            
            # Send command
            success, message = self.can_interface.send_command(0xFE, [speed & 0xFF, (speed >> 8) & 0xFF, 
                                                                     acceleration, 
                                                                     position_bytes[0], position_bytes[1], position_bytes[2]])
            
            if success:
                success, message, data = self.can_interface.receive_response()
                if success and data:
                    code = data[1]
                    if code == 0xFE:
                        status = data[2]
                        status_messages = {
                            0: "Run failed",
                            1: "Run starting",
                            2: "Run completed",
                            3: "End limit stopped"
                        }
                        self.statusBar().showMessage(status_messages.get(status, "Unknown status"))
                        return
            
            QMessageBox.warning(self, "Command Failed", f"Failed to run motor: {message}")
            
        elif mode_index == 2:  # Relative by Axis
            # Convert position to bytes (int24)
            position_bytes = position.to_bytes(3, byteorder='little', signed=True)
            
            # Send command
            success, message = self.can_interface.send_command(0xF4, [speed & 0xFF, (speed >> 8) & 0xFF, 
                                                                     acceleration, 
                                                                     position_bytes[0], position_bytes[1], position_bytes[2]])
            
            if success:
                success, message, data = self.can_interface.receive_response()
                if success and data:
                    code = data[1]
                    if code == 0xF4:
                        status = data[2]
                        status_messages = {
                            0: "Run failed",
                            1: "Run starting",
                            2: "Run completed",
                            3: "End limit stopped"
                        }
                        self.statusBar().showMessage(status_messages.get(status, "Unknown status"))
                        return
            
            QMessageBox.warning(self, "Command Failed", f"Failed to run motor: {message}")
            
        elif mode_index == 3:  # Absolute by Axis
            # Convert position to bytes (int24)
            position_bytes = position.to_bytes(3, byteorder='little', signed=True)
            
            # Send command
            success, message = self.can_interface.send_command(0xF5, [speed & 0xFF, (speed >> 8) & 0xFF, 
                                                                     acceleration, 
                                                                     position_bytes[0], position_bytes[1], position_bytes[2]])
            
            if success:
                success, message, data = self.can_interface.receive_response()
                if success and data:
                    code = data[1]
                    if code == 0xF5:
                        status = data[2]
                        status_messages = {
                            0: "Run failed",
                            1: "Run starting",
                            2: "Run completed",
                            3: "End limit stopped"
                        }
                        self.statusBar().showMessage(status_messages.get(status, "Unknown status"))
                        return
            
            QMessageBox.warning(self, "Command Failed", f"Failed to run motor: {message}")
    
    def stop_motor_position(self):
        """Stop the motor in position mode"""
        mode_index = self.position_mode_combo.currentIndex()
        acceleration = self.position_acceleration_spin.value()
        
        # Prepare command based on position mode
        if mode_index == 0:  # Relative by Pulses
            # Send stop command
            success, message = self.can_interface.send_command(0xFD, [0, 0, acceleration, 0, 0, 0])
            
            if success:
                success, message, data = self.can_interface.receive_response()
                if success and data:
                    code = data[1]
                    if code == 0xFD:
                        status = data[2]
                        status_messages = {
                            0: "Stop failed",
                            1: "Stopping",
                            2: "Stop completed",
                            3: "End limit stopped"
                        }
                        self.statusBar().showMessage(status_messages.get(status, "Unknown status"))
                        return
            
            QMessageBox.warning(self, "Command Failed", f"Failed to stop motor: {message}")
            
        elif mode_index == 1:  # Absolute by Pulses
            # Send stop command
            success, message = self.can_interface.send_command(0xFE, [0, 0, acceleration, 0, 0, 0])
            
            if success:
                success, message, data = self.can_interface.receive_response()
                if success and data:
                    code = data[1]
                    if code == 0xFE:
                        status = data[2]
                        status_messages = {
                            0: "Stop failed",
                            1: "Stopping",
                            2: "Stop completed",
                            3: "End limit stopped"
                        }
                        self.statusBar().showMessage(status_messages.get(status, "Unknown status"))
                        return
            
            QMessageBox.warning(self, "Command Failed", f"Failed to stop motor: {message}")
            
        elif mode_index == 2:  # Relative by Axis
            # Send stop command
            success, message = self.can_interface.send_command(0xF4, [0, 0, acceleration, 0, 0, 0])
            
            if success:
                success, message, data = self.can_interface.receive_response()
                if success and data:
                    code = data[1]
                    if code == 0xF4:
                        status = data[2]
                        status_messages = {
                            0: "Stop failed",
                            1: "Stopping",
                            2: "Stop completed",
                            3: "End limit stopped"
                        }
                        self.statusBar().showMessage(status_messages.get(status, "Unknown status"))
                        return
            
            QMessageBox.warning(self, "Command Failed", f"Failed to stop motor: {message}")
            
        elif mode_index == 3:  # Absolute by Axis
            # Send stop command
            success, message = self.can_interface.send_command(0xF5, [0, 0, acceleration, 0, 0, 0])
            
            if success:
                success, message, data = self.can_interface.receive_response()
                if success and data:
                    code = data[1]
                    if code == 0xF5:
                        status = data[2]
                        status_messages = {
                            0: "Stop failed",
                            1: "Stopping",
                            2: "Stop completed",
                            3: "End limit stopped"
                        }
                        self.statusBar().showMessage(status_messages.get(status, "Unknown status"))
                        return
            
            QMessageBox.warning(self, "Command Failed", f"Failed to stop motor: {message}")
    
    # Data update methods
    
    def update_encoder_data(self, data):
        """Update encoder data display"""
        value = data.get("value", 0)
        self.encoder_value_label.setText(str(value))
        
        # Calculate angle in degrees (one rotation is 0x4000 or 16384 in decimal)
        angle = (value % 16384) * 360 / 16384
        self.angle_label.setText(f"{angle:.1f}°")
    
    def update_speed_data(self, data):
        """Update speed data display"""
        speed = data.get("speed", 0)
        self.current_speed_label.setText(str(speed))
        self.speed_bar.setValue(speed)
    
    def update_io_data(self, data):
        """Update IO status display"""
        # Update IN_1
        if data.get("IN_1", False):
            self.in1_indicator.setText("ON")
            self.in1_indicator.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.in1_indicator.setText("OFF")
            self.in1_indicator.setStyleSheet("color: red; font-weight: bold;")
        
        # Update IN_2
        if data.get("IN_2", False):
            self.in2_indicator.setText("ON")
            self.in2_indicator.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.in2_indicator.setText("OFF")
            self.in2_indicator.setStyleSheet("color: red; font-weight: bold;")
        
        # Update OUT_1
        if data.get("OUT_1", False):
            self.out1_indicator.setText("ON")
            self.out1_indicator.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.out1_indicator.setText("OFF")
            self.out1_indicator.setStyleSheet("color: red; font-weight: bold;")
        
        # Update OUT_2
        if data.get("OUT_2", False):
            self.out2_indicator.setText("ON")
            self.out2_indicator.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.out2_indicator.setText("OFF")
            self.out2_indicator.setStyleSheet("color: red; font-weight: bold;")
    
    def update_error_data(self, data):
        """Update error data display"""
        error_degrees = data.get("error_degrees", 0)
        self.angle_error_label.setText(f"{error_degrees:.2f}°")
    
    def closeEvent(self, event):
        """Handle window close event"""
        # Stop data monitor thread
        if self.data_monitor:
            self.data_monitor.stop()
        
        # Disconnect from CAN bus
        if self.can_interface.connected:
            self.can_interface.disconnect()
        
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MKSConfiguratorApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
