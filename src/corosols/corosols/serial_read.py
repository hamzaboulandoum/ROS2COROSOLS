import math
import threading
import time
import rclpy
from rclpy.node import Node
import numpy as np
from custom_interfaces.msg import SerialData  # type: ignore
from custom_interfaces.srv import Commands  # type: ignore
import serial

Stepper_X_LIMIT = 2930
Stepper_Y_LIMIT = 220
AXIS_X_LIMIT = 0.150
AXIS_Y_LIMIT = 0.06
serial_port = "/dev/ttyACM0"
serial_port_stepper = "/dev/ttyACM1"
baud_rate = 115200

MOTOR_X_OFFSET = 285.0
MOTOR_Y_POS = 0.0

ROD1_init_L = 352
ROD2_init_L = 370

M1_POS = (-MOTOR_X_OFFSET, MOTOR_Y_POS)
M2_POS = (MOTOR_X_OFFSET, MOTOR_Y_POS)
AIRBRUSH_OFFSET = 35.0
AIRBRUSH_TO_ROBOT_CENTER_OFFSET_Y = 197.5  # 175mm
AIRBRUSH_TO_ROBOT_CENTER_OFFSET_X = -6.84
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D


class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')

        self.ser = serial.Serial(
            port=serial_port,
            baudrate=921600,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.005,
            write_timeout=None,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )

        self.ser.dtr = False
        self.ser.rts = False
        self.ser_stepper = serial.Serial(
            port=serial_port_stepper,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.003,
            write_timeout=None,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.flush()

        self.ser_stepper.reset_input_buffer()
        self.ser_stepper.reset_output_buffer()
        self.ser_stepper.flush()

        self.publisher_ = self.create_publisher(SerialData, 'robot_data', 10)
        
        # Use topic subscription instead of service for non-blocking commands
        self.command_subscription = self.create_subscription(
            Commands.Request,
            'commands_topic',
            self.command_callback,
            10)

        self.timer = self.create_timer(0.005, self.STM_callback)
        self.timer2 = self.create_timer(0.003, self.stepper_callback)
        self.publish_timer = self.create_timer(0.005, self.publish_robot_data)

        self.Stepper_X = 0.0
        self.Stepper_Y = 0.0
        self.msg = SerialData()
        self.time = time.time()
        self.last_stm_command = None
        self.last_stm_command2 = None
        self.stepper_buffer = bytearray()  # Buffer for stepper data

    def publish_robot_data(self):
        """Publish robot data periodically"""
        self.publisher_.publish(self.msg)

    def command_callback(self, request):
        """Handle incoming command messages (replaces service callback)"""
        if self.ser.is_open and self.ser_stepper.is_open:
            command_bytes = generate_command_bytes(
                request.vx,
                request.vy,
                request.vr,
                request.stepperx,
                request.steppery,
                request.airbrush,
                request.ab_servo,
            )
            # Only send STM command if it's different from the last one
            command_tuple = tuple(command_bytes)
            if command_tuple != self.last_stm_command:
                try:
                    self.ser.write(bytes(command_bytes))
                    self.last_stm_command = command_tuple
                except Exception as e:
                    self.get_logger().error(f'STM write error: {e}')

            # Always send stepper commands
            x = -np.clip(request.stepperx * 1000-AIRBRUSH_TO_ROBOT_CENTER_OFFSET_X, -150, 150)
            y = np.clip(
                request.steppery * 1000 +AIRBRUSH_TO_ROBOT_CENTER_OFFSET_Y , 40, 260
            )
            l1, l2 = self.inverse_kinematics(x, y)
            l1 -= ROD1_init_L
            l2 -= ROD2_init_L
            try:
                #self.get_logger().info(f'command s1: {request.stepperx} s2: {request.steppery}')
                stm2_command_bytes = generate_command_bytes_stm2(
                    l1*100,
                    l2*100,
                    request.airbrush,
                    request.ab_servo,
                    )
                command_tuple = tuple(stm2_command_bytes)
                if command_tuple != self.last_stm_command2:
                    self.ser_stepper.write(stm2_command_bytes)
                    self.last_stm_command2=command_tuple
            except Exception as e:
                self.get_logger().info(f'Stepper write error: {e}')

    def stepper_callback(self):
        """Read stepper feedback - binary packet format: {l1_hi, l1_lo, l2_hi, l2_lo, checksum, }"""
        try:
            if self.ser_stepper.is_open:
                received_char = self.ser_stepper.read(1)
                if received_char == b'{':
                    received_data = self.ser_stepper.read(7)
                    received_data = received_char + received_data
                    if len(received_data) < 8:
                        return
                        
                    Frame_Header = received_data[0]
                    Flag_Stop = received_data[1]
                    l1_int = (received_data[2] << 8) | received_data[3]
                    l2_int = (received_data[4] << 8) | received_data[5]
                    
                    # Convert to signed
                    l1_int = self.convert_to_signed(l1_int)
                    l2_int = self.convert_to_signed(l2_int)
                    # Convert back to float (divide by 100)
                    l1 = l1_int / 100.0
                    l2 = l2_int / 100.0
                    
                    # Forward kinematics calculation (add init lengths back)
                    result = self.forward_kinematics(l1 + ROD1_init_L, l2 + ROD2_init_L)
                    Checksum = received_data[6]
                    Frame_Tail = received_data[7]

                    if True:#Check_Sum(received_data, 6) == Checksum:
                        if result:
                            stepper_x, stepper_y = result
                            self.msg.stepper_x = -stepper_x
                            self.msg.stepper_y = stepper_y
                            #self.get_logger().info(f'Stepper 1: {self.msg.stepper_x}, stepper 2: {self.msg.stepper_y}')
            '''# Read all available bytes
            while self.ser_stepper.in_waiting > 0:
                byte = self.ser_stepper.read(1)
                if byte:
                    self.stepper_buffer.extend(byte)
                    
                    # Look for complete packet: { (header) + 4 data bytes + checksum + } (tail) = 7 bytes
                    while len(self.stepper_buffer) >= 7:
                        # Find header byte '{'
                        if self.stepper_buffer[0] != 0x7B:  # '{'
                            self.stepper_buffer.pop(0)
                            continue
                        
                        # Check if we have tail byte '}' at position 6
                        if self.stepper_buffer[6] != 0x7D:  # '}'
                            self.stepper_buffer.pop(0)
                            continue
                        
                        # Extract packet
                        packet = self.stepper_buffer[:7]
                        self.stepper_buffer = self.stepper_buffer[7:]
                        
                        # Verify checksum (XOR of bytes 1-4)
                        checksum = 0
                        for i in range(1, 5):
                            checksum ^= packet[i]
                        
                        if checksum != packet[5]:
                            self.get_logger().warn(f'Stepper checksum error: got {packet[5]}, expected {checksum}')
                            continue'''
                        
                        # Parse l1 and l2 (signed 16-bit, scaled by 100)
                        
                        
        except serial.SerialException as e:
            self.get_logger().error(f'Stepper serial error: {e}')
        except Exception as e:
            self.get_logger().error(f'Stepper callback error: {e}')

    @staticmethod
    def forward_kinematics(l1, l2):
        x1, y1 = M1_POS
        x2, y2 = M2_POS

        d = math.hypot(x2 - x1, y2 - y1)

        if d > l1 + l2 or d < abs(l1 - l2) or d == 0:
            return None

        a = (l1**2 - l2**2 + d**2) / (2 * d)
        h = math.sqrt(max(0, l1**2 - a**2))

        x_mid = x1 + a * (x2 - x1) / d
        y_mid = y1 + a * (y2 - y1) / d

        P_x = x_mid + h * (y2 - y1) / d
        P_y = y_mid + h * (x2 - x1) / d

        angle_rod2 = math.atan2(P_y - y2, P_x - x2)

        airbrush_x = P_x + 35 * math.cos(angle_rod2 + math.pi / 2)
        airbrush_y = P_y + 35 * math.sin(angle_rod2 + math.pi / 2)

        return (airbrush_x-AIRBRUSH_TO_ROBOT_CENTER_OFFSET_X)/ 1000.0, (airbrush_y - AIRBRUSH_TO_ROBOT_CENTER_OFFSET_Y) / 1000.0

    @staticmethod
    def inverse_kinematics(x_airbrush, y_airbrush):
        dx = x_airbrush - M2_POS[0]
        dy = y_airbrush - M2_POS[1]

        d_airbrush = math.hypot(dx, dy)

        if d_airbrush < AIRBRUSH_OFFSET:
            return ROD1_init_L, ROD2_init_L

        d_rod2 = math.sqrt(d_airbrush**2 - AIRBRUSH_OFFSET**2)

        angle_to_pen = math.atan2(dy, dx)

        angle_offset = math.atan2(AIRBRUSH_OFFSET, d_rod2)
        angle_rod2 = angle_to_pen - angle_offset

        P_x = M2_POS[0] + d_rod2 * math.cos(angle_rod2)
        P_y = M2_POS[1] + d_rod2 * math.sin(angle_rod2)

        l1 = math.hypot(P_x - M1_POS[0], P_y - M1_POS[1])
        l2 = d_rod2

        return l1, l2

    def STM_callback(self):
        if self.ser.is_open:
            received_char = self.ser.read(1)
            if received_char == b'{':
                received_data = self.ser.read(27)
                received_data = received_char + received_data
                if len(received_data) < 28:
                    return
                    
                Frame_Header = received_data[0]
                Flag_Stop = received_data[1]

                X_speed = self.convert_to_signed(
                    (received_data[2] << 8) | received_data[3]
                )
                Y_speed = self.convert_to_signed(
                    (received_data[4] << 8) | received_data[5]
                )
                Z_speed = self.convert_to_signed(
                    (received_data[6] << 8) | received_data[7]
                )

                X_accel = self.convert_to_signed(
                    (received_data[8] << 8) | received_data[9]
                )
                Y_accel = self.convert_to_signed(
                    (received_data[10] << 8) | received_data[11]
                )
                Z_accel = self.convert_to_signed(
                    (received_data[12] << 8) | received_data[13]
                )

                Heading = (received_data[14] << 8) | received_data[15]
                Pitch = self.convert_to_signed(
                    (received_data[16] << 8) | received_data[17]
                )
                Roll = self.convert_to_signed(
                    (received_data[18] << 8) | received_data[19]
                )

                Power_Voltage = self.convert_to_signed(
                    (received_data[20] << 8) | received_data[21]
                )
                Heading_IMU = self.convert_to_signed(
                    (received_data[22] << 8) | received_data[23]
                )

                Checksum = received_data[26]
                Frame_Tail = received_data[27]

                if Check_Sum(received_data, 26) == Checksum:
                    self.msg.x_speed = -float(Y_speed / 1e6)
                    self.msg.y_speed = float(X_speed / 1e6)
                    self.msg.z_speed = float(Z_speed / 1e6)

                    self.msg.x_accel = X_accel / 16384.0 * 9.81
                    self.msg.y_accel = Y_accel / 16384.0 * 9.81
                    self.msg.z_accel = Z_accel / 16384.0 * 9.81

                    self.msg.heading = Heading_IMU/100.0
                    self.msg.pitch = Pitch / 100.0
                    self.msg.roll = Roll / 100.0

                    self.msg.power_voltage = Power_Voltage / 1000

    def convert_to_signed(self, value):
        if value & 0x8000:
            return value - 0x10000
        return value


def Check_Sum(data, count_number):
    check_sum = 0
    for k in range(count_number):
        check_sum ^= data[k]
    return check_sum


def generate_command_bytes(
    velocity_x, velocity_y, angular_z, stepperx, steppery, compressor1, ab_servo
):
    velocity_x = int(float(velocity_x) * 1000)
    velocity_y = int(float(velocity_y) * 1000)
    angular_z = int(float(angular_z) * 1000)
    velocity_x = max(-32768, min(32767, velocity_x))
    velocity_y = max(-32768, min(32767, velocity_y))
    angular_z = max(-32768, min(32767, angular_z))
    stepperx = 0
    steppery = 0

    compressor1 = int(compressor1 * 15000 / 100)
    ab_servo = int(2150 + ab_servo * 450 / 100)

    if velocity_x < 0:
        velocity_x = velocity_x & 0xFFFF
    if velocity_y < 0:
        velocity_y = velocity_y & 0xFFFF
    if angular_z < 0:
        angular_z = angular_z & 0xFFFF

    stepperx = int(stepperx) & 0xFFFF
    steppery = int(steppery) & 0xFFFF
    compressor1 = int(compressor1) & 0xFFFF
    ab_servo = int(ab_servo) & 0xFFFF

    command = [
        FRAME_HEADER,
        0x00,
        0x00,
        velocity_x >> 8 & 0xFF,
        velocity_x & 0xFF,
        velocity_y >> 8 & 0xFF,
        velocity_y & 0xFF,
        angular_z >> 8 & 0xFF,
        angular_z & 0xFF,
        steppery >> 8 & 0xFF,
        steppery & 0xFF,
        stepperx >> 8 & 0xFF,
        stepperx & 0xFF,
        compressor1 >> 8 & 0xFF,
        compressor1 & 0xFF,
        ab_servo >> 8 & 0xFF,
        ab_servo & 0xFF,
        0x00,
        FRAME_TAIL,
    ]

    checksum = Check_Sum(command, 17)
    command[17] = checksum
    return command

def generate_command_bytes_stm2(stepperx, steppery, compressor1, ab_servo):

    compressor1 = int(compressor1 * 15000 / 100)
    ab_servo = int(2150 + ab_servo * 450 / 100)


    stepperx = int(stepperx) & 0xFFFF
    steppery = int(steppery) & 0xFFFF
    compressor1 = int(compressor1) & 0xFFFF
    ab_servo = int(ab_servo) & 0xFFFF

    command = [
        FRAME_HEADER,
        0x00,
        stepperx >> 8 & 0xFF,
        stepperx & 0xFF,
        steppery >> 8 & 0xFF,
        steppery & 0xFF,
        compressor1 >> 8 & 0xFF,
        compressor1 & 0xFF,
        ab_servo >> 8 & 0xFF,
        ab_servo & 0xFF,
        0x00,
        FRAME_TAIL,
    ]

    checksum = Check_Sum(command, 10)
    command[10] = checksum
    return command

def main(args=None):
    rclpy.init(args=None)
    serial_reader = SerialReader()
    try:
        rclpy.spin(serial_reader)
    except KeyboardInterrupt:
        serial_reader.get_logger().info("Interrupted by user, shutting down")
        command_bytes = generate_command_bytes(0, 0, 0, 0, 0, 0, 0)
        serial_reader.ser.write(bytes(command_bytes))
        serial_reader.ser.close()
        serial_reader.destroy_node()


if __name__ == '__main__':
    main()