import math
import threading
import time
import csv
import os
import rclpy
from rclpy.node import Node
import numpy as np
from custom_interfaces.msg import SerialData  # type: ignore
from custom_interfaces.srv import Commands  # type: ignore
import serial

Stepper_X_LIMIT = 2930
Stepper_Y_LIMIT = 220
AXIS_X_LIMIT = 0.21
AXIS_Y_LIMIT_NEGATIVE = 0.100 #was 0.06
AXIS_Y_LIMIT_POSITIVE = 0.160
serial_port = "/dev/ttyACM1"
serial_port_stepper = "/dev/ttyACM0"
baud_rate = 115200

MOTOR_X_OFFSET = 319.5#285.0
MOTOR_Y_POS = 0.0

ROD1_init_L = 374#352
ROD2_init_L = 359#370

M1_POS = (MOTOR_X_OFFSET, MOTOR_Y_POS)
M2_POS = (-MOTOR_X_OFFSET, MOTOR_Y_POS)
AIRBRUSH_OFFSET = 33.5#35.0
AIRBRUSH_TO_ROBOT_CENTER_OFFSET_Y = 197.5  # 175mm
AIRBRUSH_TO_ROBOT_CENTER_OFFSET_X = 0#-6.84
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D


class SerialReader(Node):
    def __init__(self):
        super().__init__('serial_reader')

        # --- CSV logging for transmitted stepper commands ---
        log_dir = os.path.expanduser('~/Desktop/Application/COROSOLS_WS')
        self.stepper_log_path = os.path.join(log_dir, 'stepper_commands_log.csv')
        self.stepper_log_file = open(self.stepper_log_path, 'w', newline='')
        self.stepper_log_writer = csv.writer(self.stepper_log_file)
        self.stepper_log_writer.writerow([
            'timestamp', 'stepperx', 'steppery', 'l1_mm', 'l2_mm', 'l1_tx', 'l2_tx',
            'feedback_l1_mm', 'feedback_l2_mm', 'predicted_l1_mm', 'predicted_l2_mm'
        ])
        self.stepper_log_start_time = time.time()

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

        self.ser_stepper.dtr = False
        self.ser_stepper.rts = False

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
        
        # Compute the true home position (FK at l1=ROD1_init_L, l2=ROD2_init_L)
        # When STM32 reports feedback delta = 0, rods are at init lengths.
        # FK of those lengths is NOT (0,0) in airbrush coordinates.
        home_result = self.forward_kinematics(ROD1_init_L, ROD2_init_L)
        if home_result:
            home_x, home_y = home_result
            self.msg.stepper_x = home_x
            self.msg.stepper_y = home_y
            self.get_logger().info(f'Stepper home position: x={home_x*1000:.2f}mm, y={home_y*1000:.2f}mm')
        self.time = time.time()
        self.last_stm_command = None
        self.last_stm_command2 = None
        self.stepper_buffer = bytearray()  # Buffer for stepper data

        # --- Stepper prediction model ---
        # Predicted rod lengths (delta from init, same units as commands: hundredths of mm)
        self.predicted_l1 = 0.0  # delta from ROD1_init_L, in mm
        self.predicted_l2 = 0.0  # delta from ROD2_init_L, in mm
        # Last commanded rod lengths (delta from init, in mm)
        self.commanded_l1 = 0.0
        self.commanded_l2 = 0.0
        # Stepper rod speed in mm per second (0.8 m/s = 800 mm/s)
        self.stepper_speed = 800.0
        # Feedback correction: blend factor (0 = pure prediction, 1 = pure feedback)
        self.feedback_blend = 0.15
        self.prediction_time = time.time()
        # Track printing state for full-sync on transitions
        self.is_printing = False
        # Last raw feedback from STM32 (for full-sync on print→no-print)
        self.last_feedback_l1 = 0.0
        self.last_feedback_l2 = 0.0
        self.has_feedback = False  # True once we've received at least one feedback packet

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
            x = np.clip(request.stepperx * 1000-AIRBRUSH_TO_ROBOT_CENTER_OFFSET_X, -AXIS_X_LIMIT*1000, AXIS_X_LIMIT*1000)
            y = np.clip(
                request.steppery * 1000 + AIRBRUSH_TO_ROBOT_CENTER_OFFSET_Y,
                -AXIS_Y_LIMIT_NEGATIVE*1000 + AIRBRUSH_TO_ROBOT_CENTER_OFFSET_Y,
                AXIS_Y_LIMIT_POSITIVE*1000 + AIRBRUSH_TO_ROBOT_CENTER_OFFSET_Y
            )
            l1, l2 = self.inverse_kinematics(x, y)
            l1 -= ROD1_init_L
            l2 -= ROD2_init_L
            # Store commanded rod lengths for prediction model (in mm)
            self.commanded_l1 = l1
            self.commanded_l2 = l2
            try:
                #self.get_logger().info(f'command s1: {request.stepperx} s2: {request.steppery}')
                stm2_command_bytes = generate_command_bytes_stm2(
                    l1*100,
                    l2*100,
                    request.airbrush,
                    request.ab_servo,
                    )
                # Always send stepper commands - no dedup filtering.
                # The STM32 interprets each command as a target position.
                # Filtering caused jitter: the STM32 would start moving,
                # then get no command (filtered as "same"), lose its motion,
                # then get a new slightly-different command and restart.
                self.ser_stepper.write(stm2_command_bytes)

                # Log transmitted stepper values to CSV
                elapsed = time.time() - self.stepper_log_start_time
                self.stepper_log_writer.writerow([
                    f'{elapsed:.4f}',
                    f'{request.stepperx:.6f}', f'{request.steppery:.6f}',
                    f'{l1:.3f}', f'{l2:.3f}',
                    f'{l1*100:.1f}', f'{l2*100:.1f}',
                    f'{self.last_feedback_l1:.3f}', f'{self.last_feedback_l2:.3f}',
                    f'{self.predicted_l1:.3f}', f'{self.predicted_l2:.3f}',
                ])
                self.stepper_log_file.flush()
            except Exception as e:
                self.get_logger().info(f'Stepper write error: {e}')

    def stepper_callback(self):
        """Read stepper feedback from STM32 and publish position via FK."""
        try:
            if self.ser_stepper.is_open:
                received_char = self.ser_stepper.read(1)
                if received_char == b'{':
                    received_data = self.ser_stepper.read(11)
                    received_data = received_char + received_data
                    if len(received_data) >= 12:
                        l1_int = (received_data[2] << 24) | (received_data[3] << 16) | (received_data[4] << 8) | received_data[5]
                        l2_int = (received_data[6] << 24) | (received_data[7] << 16) | (received_data[8] << 8) | received_data[9]
                        l1_int = self.convert_to_signed_32(l1_int)
                        l2_int = self.convert_to_signed_32(l2_int)
                        feedback_l1 = l1_int / 100.0  # delta from init, in mm
                        feedback_l2 = l2_int / 100.0

                        self.last_feedback_l1 = feedback_l1
                        self.last_feedback_l2 = feedback_l2

                        result = self.forward_kinematics(
                            feedback_l1 + ROD1_init_L,
                            feedback_l2 + ROD2_init_L
                        )
                        if result:
                            stepper_x, stepper_y = result
                            self.msg.stepper_x = stepper_x
                            self.msg.stepper_y = stepper_y

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
        P_y = y_mid - h * (x2 - x1) / d

        angle_rod2 = math.atan2(P_y - y2, P_x - x2)

        airbrush_x = P_x + AIRBRUSH_OFFSET * math.cos(angle_rod2 + math.pi / 2)
        airbrush_y = P_y + AIRBRUSH_OFFSET * math.sin(angle_rod2 + math.pi / 2)

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

    def convert_to_signed_32(self, value):
        if value & 0x80000000:
            return value - 0x100000000
        return value


def Check_Sum(data, count_number):
    check_sum = 0
    for k in range(count_number):
        check_sum ^= data[k]
    return check_sum


def generate_command_bytes(
    velocity_x, velocity_y, angular_z, stepperx, steppery, compressor1, linear_actuator
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
    linear_actuator = int(2150 + linear_actuator * 450 / 100)

    if velocity_x < 0:
        velocity_x = velocity_x & 0xFFFF
    if velocity_y < 0:
        velocity_y = velocity_y & 0xFFFF
    if angular_z < 0:
        angular_z = angular_z & 0xFFFF

    stepperx = int(stepperx) & 0xFFFF
    steppery = int(steppery) & 0xFFFF
    compressor1 = int(compressor1) & 0xFFFF
    linear_actuator = int(linear_actuator) & 0xFFFF

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
        linear_actuator >> 8 & 0xFF,
        linear_actuator & 0xFF,
        0x00,
        FRAME_TAIL,
    ]

    checksum = Check_Sum(command, 17)
    command[17] = checksum
    return command

def generate_command_bytes_stm2(stepperx, steppery, compressor1, ab_servo):

    compressor1 = int(compressor1 * 15000 / 100)
    ab_servo = int(2150 + ab_servo * 450 / 100)


    stepperx = int(stepperx) & 0xFFFFFFFF
    steppery = int(steppery) & 0xFFFFFFFF
    compressor1 = int(compressor1) & 0xFFFF
    ab_servo = int(ab_servo) & 0xFFFF

    command = [
        FRAME_HEADER,
        0x00,
        (stepperx >> 24) & 0xFF,
        (stepperx >> 16) & 0xFF,
        (stepperx >> 8) & 0xFF,
        stepperx & 0xFF,
        (steppery >> 24) & 0xFF,
        (steppery >> 16) & 0xFF,
        (steppery >> 8) & 0xFF,
        steppery & 0xFF,
        compressor1 >> 8 & 0xFF,
        compressor1 & 0xFF,
        ab_servo >> 8 & 0xFF,
        ab_servo & 0xFF,
        0x00,
        FRAME_TAIL,
    ]

    checksum = Check_Sum(command, 14)
    command[14] = checksum
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