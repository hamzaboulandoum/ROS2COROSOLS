#!/usr/bin/env python3

import serial
import threading
import struct
import rclpy
from rclpy.node import Node
from custom_interfaces.msg import ImuData
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE

class SerialDataPublisher(Node):
    def __init__(self):
        super().__init__('imu_data_publisher')
        
        # Constants for frame parsing
        self.FRAME_HEAD = 'fc'
        self.TYPE_IMU = '61'
        self.TYPE_AHRS = '41'
        self.TYPE_INSGPS = '42'
        self.TYPE_GEODETIC_POS = '5c'
        self.TYPE_SYS_STATE = '50'
        
        self.FRAME_LENGTHS = {
            self.TYPE_IMU: 0x56,          # 86 bytes
            self.TYPE_AHRS: 0x30,         # 48 bytes
            self.TYPE_INSGPS: 0x48,       # 72 bytes
            self.TYPE_GEODETIC_POS: 0x20, # 32 bytes
            self.TYPE_SYS_STATE: 0x64     # 100 bytes
        }
        
        # Serial port configuration
        PORT = '/dev/ttyACM0'
        BAUDRATE = 921600
        
        self.get_logger().info(f"Opening serial port {PORT} at {BAUDRATE} baud")
        
        try:
            self.ser = serial.Serial(
                port=PORT,
                baudrate=BAUDRATE,
                bytesize=EIGHTBITS,
                parity=PARITY_NONE,
                stopbits=STOPBITS_ONE,
                timeout=0.1
            )
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # Clear buffers
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
        self.publisher_ = self.create_publisher(ImuData, 'imu_data', 10)
        self.imu_data = ImuData()
        
        # Start reading thread
        self.running = True
        self.thread = threading.Thread(target=self.read_serial_data)
        self.thread.daemon = True
        self.thread.start()

    def synchronize(self):
        """Find the next frame header in the serial stream"""
        while self.running:
            try:
                byte = self.ser.read(1)
                if not byte:
                    continue
                if byte.hex() == self.FRAME_HEAD:
                    return True
            except serial.SerialException as e:
                self.get_logger().error(f"Serial error during sync: {e}")
                return False
        return False

    def read_frame(self):
        """Read and process a complete frame"""
        try:
            # Read frame type
            frame_type = self.ser.read(1)
            if not frame_type:
                return False
            
            # Read length
            length_byte = self.ser.read(1)
            if not length_byte:
                return False
                
            # Read header (SN, CRC8, CRC16_H, CRC16_L)
            header = self.ser.read(4)
            if len(header) != 4:
                return False

            frame_type_hex = frame_type.hex()
            frame_length = self.FRAME_LENGTHS.get(frame_type_hex, 0)
            
            if frame_length == 0:
                self.get_logger().warning(f"Unknown frame type: {frame_type_hex}")
                return False

            # Read the data portion
            data = self.ser.read(frame_length)
            if len(data) != frame_length:
                self.get_logger().warning(f"Incomplete frame data: expected {frame_length}, got {len(data)}")
                return False
            self.get_logger().warning(f"{frame_type_hex}")
            # Process different frame types
            if frame_type_hex == self.TYPE_IMU:
                self.process_imu_data(data)
            elif frame_type_hex == self.TYPE_AHRS:
                self.process_ahrs_data(data)
            elif frame_type_hex == self.TYPE_INSGPS:
                self.process_insgps_data(data)
            elif frame_type_hex == self.TYPE_GEODETIC_POS:
                self.process_geodetic_data(data)
            elif frame_type_hex == self.TYPE_SYS_STATE:
                self.process_sys_state_data(data)
            
            return True

        except Exception as e:
            self.get_logger().error(f"Error reading frame: {e}")
            return False

    def process_imu_data(self, data):
        """Process IMU data frame and publish to ROS topic"""
        try:
            msg = ImuData()
            msg.gyro_x = struct.unpack('f', data[0:4])[0]
            msg.gyro_y = struct.unpack('f', data[4:8])[0]
            msg.gyro_z = struct.unpack('f', data[8:12])[0]
            msg.accel_x = struct.unpack('f', data[12:16])[0]
            msg.accel_y = struct.unpack('f', data[16:20])[0]
            msg.accel_z = struct.unpack('f', data[20:24])[0]
            msg.mag_x = struct.unpack('f', data[24:28])[0]
            msg.mag_y = struct.unpack('f', data[28:32])[0]
            msg.mag_z = struct.unpack('f', data[32:36])[0]
            
            # Additional IMU data logging
            temperature = struct.unpack('f', data[36:40])[0]
            pressure = struct.unpack('f', data[40:44])[0]
            pressure_temp = struct.unpack('f', data[44:48])[0]
            timestamp = struct.unpack('i', data[48:52])[0]
            
            self.get_logger().info(
                f"IMU Data:\n"
                f"  Gyro (rad/s): {msg.gyro_x:.2f}, {msg.gyro_y:.2f}, {msg.gyro_z:.2f}\n"
                f"  Accel (m/s²): {msg.accel_x:.2f}, {msg.accel_y:.2f}, {msg.accel_z:.2f}\n"
                f"  Mag (mG): {msg.mag_x:.2f}, {msg.mag_y:.2f}, {msg.mag_z:.2f}\n"
                f"  Temp: {temperature:.2f}°C, Pressure: {pressure:.2f}, Pressure Temp: {pressure_temp:.2f}\n"
                f"  Timestamp: {timestamp} us"
            )
            
            self.publisher_.publish(msg)
            
        except struct.error as e:
            self.get_logger().error(f"Error unpacking IMU data: {e}")

    def process_ahrs_data(self, data):
        """Process AHRS data frame"""
        try:
            roll_speed = struct.unpack('f', data[0:4])[0]
            pitch_speed = struct.unpack('f', data[4:8])[0]
            heading_speed = struct.unpack('f', data[8:12])[0]
            roll = struct.unpack('f', data[12:16])[0]
            pitch = struct.unpack('f', data[16:20])[0]
            heading = struct.unpack('f', data[20:24])[0]
            q1 = struct.unpack('f', data[24:28])[0]
            q2 = struct.unpack('f', data[28:32])[0]
            q3 = struct.unpack('f', data[32:36])[0]
            q4 = struct.unpack('f', data[36:40])[0]
            
            self.get_logger().info(
                f"AHRS Data:\n"
                f"  Angular Rates (rad/s) - Roll: {roll_speed:.2f}, Pitch: {pitch_speed:.2f}, Heading: {heading_speed:.2f}\n"
                f"  Angles (rad) - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Heading: {heading:.2f}\n"
                f"  Quaternion - Q1: {q1:.3f}, Q2: {q2:.3f}, Q3: {q3:.3f}, Q4: {q4:.3f}"
            )
        except struct.error as e:
            self.get_logger().error(f"Error unpacking AHRS data: {e}")

    def process_insgps_data(self, data):
        """Process INSGPS data frame"""
        try:
            vel_x = struct.unpack('f', data[0:4])[0]
            vel_y = struct.unpack('f', data[4:8])[0]
            vel_z = struct.unpack('f', data[8:12])[0]
            acc_x = struct.unpack('f', data[12:16])[0]
            acc_y = struct.unpack('f', data[16:20])[0]
            acc_z = struct.unpack('f', data[20:24])[0]
            pos_n = struct.unpack('f', data[24:28])[0]
            pos_e = struct.unpack('f', data[28:32])[0]
            pos_d = struct.unpack('f', data[32:36])[0]
            vel_n = struct.unpack('f', data[36:40])[0]
            vel_e = struct.unpack('f', data[40:44])[0]
            vel_d = struct.unpack('f', data[44:48])[0]
            
            self.get_logger().info(
                f"INSGPS Data:\n"
                f"  Body Velocity (m/s) - X: {vel_x:.2f}, Y: {vel_y:.2f}, Z: {vel_z:.2f}\n"
                f"  Body Acceleration (m/s²) - X: {acc_x:.2f}, Y: {acc_y:.2f}, Z: {acc_z:.2f}\n"
                f"  Position (m) - N: {pos_n:.2f}, E: {pos_e:.2f}, D: {pos_d:.2f}\n"
                f"  Velocity (m/s) - N: {vel_n:.2f}, E: {vel_e:.2f}, D: {vel_d:.2f}"
            )
        except struct.error as e:
            self.get_logger().error(f"Error unpacking INSGPS data: {e}")

    def process_geodetic_data(self, data):
        """Process Geodetic Position data frame"""
        try:
            latitude = struct.unpack('d', data[0:8])[0]
            longitude = struct.unpack('d', data[8:16])[0]
            height = struct.unpack('d', data[16:24])[0]
            
            self.get_logger().info(
                f"Geodetic Position:\n"
                f"  Latitude (rad): {latitude:.6f}\n"
                f"  Longitude (rad): {longitude:.6f}\n"
                f"  Height (m): {height:.2f}"
            )
        except struct.error as e:
            self.get_logger().error(f"Error unpacking geodetic data: {e}")

    def process_sys_state_data(self, data):
        """Process System State data frame"""
        try:
            system_status = struct.unpack('d', data[0:8])[0]
            self.get_logger().info(f"System State - Status: {system_status}")
        except struct.error as e:
            self.get_logger().error(f"Error unpacking system state data: {e}")

    def read_serial_data(self):
        """Main read loop"""
        while self.running:
            try:
                if self.synchronize():
                    self.read_frame()
            except Exception as e:
                self.get_logger().error(f"Error in read loop: {e}")
                continue

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()