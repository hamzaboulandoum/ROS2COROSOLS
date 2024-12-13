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
        self.TYPE_IMU = '40'
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
        PORT = '/dev/ttyACM1'
        BAUDRATE = 921600
        
        try:
            self.ser = serial.Serial(
                port=PORT,
                baudrate=BAUDRATE,
                timeout=0.01
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
            head_type = self.ser.read().hex()
            check_len = self.ser.read().hex()
            # Read header (SN, CRC8, CRC16_H, CRC16_L)
            check_sn = self.ser.read().hex()
            head_crc8 = self.ser.read().hex()
            crc16_H_s = self.ser.read().hex()
            crc16_L_s = self.ser.read().hex()

            frame_length = self.FRAME_LENGTHS.get(head_type, 0)
            
            if frame_length == 0:
                return False

            # Read the data portion
            data = self.ser.read(frame_length)
            if len(data) != frame_length:
                self.get_logger().warning(f"Incomplete frame data: expected {frame_length}, got {len(data)}")
                return False
            # Process different frame types
            if head_type == self.TYPE_IMU:
                self.process_imu_data(data)
            elif head_type == self.TYPE_AHRS:
                self.process_ahrs_data(data)
            elif head_type == self.TYPE_INSGPS:
                self.process_insgps_data(data)
            elif head_type == self.TYPE_GEODETIC_POS:
                self.process_geodetic_data(data)
            elif head_type == self.TYPE_SYS_STATE:
                self.process_sys_state_data(data)
            
            return True

        except Exception as e:
            self.get_logger().error(f"Error reading frame: {e}")
            return False

    def process_imu_data(self, data):
        """Process IMU data frame and publish to ROS topic"""
        try:
            self.imu_data.gyroscope_x = struct.unpack('f', data[0:4])[0]
            self.imu_data.gyroscope_y = struct.unpack('f', data[4:8])[0]
            self.imu_data.gyroscope_z = struct.unpack('f', data[8:12])[0]
            self.imu_data.accelerometer_x = struct.unpack('f', data[12:16])[0]
            self.imu_data.accelerometer_y = struct.unpack('f', data[16:20])[0]
            self.imu_data.accelerometer_z = struct.unpack('f', data[20:24])[0]
            self.imu_data.magnetometer_x = struct.unpack('f', data[24:28])[0]
            self.imu_data.magnetometer_y = struct.unpack('f', data[28:32])[0]
            self.imu_data.magnetometer_z = struct.unpack('f', data[32:36])[0]
            
            # Additional IMU data logging
            self.imu_data.temperature = struct.unpack('f', data[36:40])[0]
            self.imu_data.pressure = struct.unpack('f', data[40:44])[0]
            self.imu_data.pressure_temp = struct.unpack('f', data[44:48])[0]
            self.imu_data.timestamp = struct.unpack('i', data[48:52])[0]
            
            
            self.publisher_.publish(self.imu_data)
            
        except struct.error as e:
            self.get_logger().error(f"Error unpacking IMU data: {e}")

    def process_ahrs_data(self, data):
        """Process AHRS data frame"""
        try:
            self.imu_data.roll_speed = struct.unpack('f', data[0:4])[0]
            self.imu_data.pitch_speed = struct.unpack('f', data[4:8])[0]
            self.imu_data.heading_speed = struct.unpack('f', data[8:12])[0]
            self.imu_data.roll = struct.unpack('f', data[12:16])[0]
            self.imu_data.pitch = struct.unpack('f', data[16:20])[0]
            self.imu_data.heading = struct.unpack('f', data[20:24])[0]
            self.imu_data.q1 = struct.unpack('f', data[24:28])[0]
            self.imu_data.q2 = struct.unpack('f', data[28:32])[0]
            self.imu_data.q3 = struct.unpack('f', data[32:36])[0]
            self.imu_data.q4 = struct.unpack('f', data[36:40])[0]
            self.publisher_.publish(self.imu_data)
            
        except struct.error as e:
            self.get_logger().error(f"Error unpacking AHRS data: {e}")

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
        
        node.ser.close()
        node.destroy_node()

if __name__ == '__main__':
    main()