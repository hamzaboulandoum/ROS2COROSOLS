import time
import rclpy
from rclpy.node import Node

from custom_interfaces.srv import Commands # type: ignore
from custom_interfaces.msg import SerialData # type: ignore
import serial


Stepper_X_LIMIT = 1125
Stepper_Y_LIMIT = 300
AXIS_X_LIMIT = 0.114
AXIS_Y_LIMIT = 0.0325
serial_port = "/dev/ttyACM0" # Update with your serial port
baud_rate = 115200
class SerialHandler(Node):
    
    def convert_to_signed(self,value):
        if value & 0x8000:
            return value - 0x10000
        return value


    def __init__(self):
        super().__init__('serial_handler')
        
        self.ser = serial.Serial(serial_port, baud_rate)
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.flush()
        
        #THIS PUBLISHES THE DATA FROM STM32 To Topic topic
        self.publisher_ = self.create_publisher(SerialData, 'robot_data', 10)
        timer_period = 0.05  # seconds        
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.velocity_srv = self.create_service(Commands, 'commands', self.send_data_to_serial)
        
    def send_data_to_serial(self, request, response):

        if self.ser.is_open:
            command_bytes = generate_command_bytes(request.vx, request.vy, request.vr,request.stepperx,request.steppery, 5000 if request.airbrush==1 else 0, 5000 if request.airbrush==1 else 0)
            try:
                self.ser.write(bytes(command_bytes))
            except KeyboardInterrupt:
                self.ser.close()
            
            #Getting commands from velocity calculator
            #self.get_logger().info('Incoming request : x: %f y: %f r: %f  s1: %f s2: %f a: %d' % (request.vx, request.vy,request.vr,request.stepperx, request.steppery, request.airbrush)) 
            
            # Sending feedback to velocity calculator to let him know it was executed.
            response.outcome = f"""Sent Succesfully"""    
            #self.get_logger().info('Request was sent successfully: outcome: %s ' % (response.outcome))     
        else:
            pass
        
        return response
    
               
    def timer_callback(self):  
           
        msg = SerialData()                 
        if self.ser.is_open:
            received_char = self.ser.read(1)
            if received_char == b'{':
                received_data = self.ser.read(27)
                
                received_data = received_char +received_data
                Frame_Header = received_data[0]
                Flag_Stop = received_data[1]
        
                X_speed = self.convert_to_signed((received_data[2] << 8) | received_data[3])
                Y_speed = self.convert_to_signed((received_data[4] << 8) | received_data[5])
                Z_speed = self.convert_to_signed((received_data[6] << 8) | received_data[7])

                X_accel = self.convert_to_signed((received_data[8] << 8) | received_data[9])
                Y_accel = self.convert_to_signed((received_data[10] << 8) | received_data[11])
                Z_accel = self.convert_to_signed((received_data[12] << 8) | received_data[13])

                X_gyro = self.convert_to_signed((received_data[14] << 8) | received_data[15])
                Y_gyro = self.convert_to_signed((received_data[16] << 8) | received_data[17])
                Z_gyro = self.convert_to_signed((received_data[18] << 8) | received_data[19])

                Power_Voltage = self.convert_to_signed((received_data[20] << 8) | received_data[21])

                Stepper_X = self.convert_to_signed((received_data[24] << 8) | received_data[25])
                Stepper_Y = self.convert_to_signed((received_data[22] << 8) | received_data[23])

                Checksum = received_data[26]
                Frame_Tail = received_data[27]
    
            
                if Check_Sum(received_data, 26, 1) == Checksum:
                    msg.x_speed = X_speed/1000 # X-axis speed (16-bit integer)        
                    msg.y_speed = Y_speed/1000
                    msg.z_speed = Z_speed/1000

                    msg.x_accel = X_accel/1000
                    msg.y_accel = Y_accel/1000
                    msg.z_accel = Z_accel/1000
                    
                    msg.x_gyro = X_gyro/1000     
                    msg.y_gyro = Y_gyro/1000
                    msg.z_gyro = Z_gyro/1000

                    msg.power_voltage = Power_Voltage/1000
                    msg.stepper_x = (Stepper_X/Stepper_X_LIMIT)*(AXIS_X_LIMIT*2)-AXIS_X_LIMIT
                    msg.stepper_y = (Stepper_Y/Stepper_Y_LIMIT)*(AXIS_Y_LIMIT*2)-AXIS_Y_LIMIT
                    
                    
                    self.publisher_.publish(msg)
        else:
            pass
      
    def listener_callback(self, msg):

        command_bytes = generate_command_bytes(msg.vx, msg.vy,msg.vr,msg.airbrush)
        for byte in command_bytes:
            self.ser.write(bytes([byte]))
            time.sleep(0.001) #-----  Why do we have this maybe needs to be removed since we are already calling the function multiple times.
        #self.get_logger().info(f"{msg}")          


def Check_Sum(data, count_number, mode):
    check_sum = 0
    # Validate the data for checksum calculation
    for k in range(count_number):
        if mode == 1:
            # Calculate checksum for sending data
            check_sum ^= data[k]  # XOR operation for sending data
        elif mode == 0:
            # Calculate checksum for received data
            check_sum ^= data[k]  # XOR operation for received data

    return check_sum

def generate_command_bytes(velocity_x, velocity_y, angular_z, stepperx, steppery, compressor1, compressor2):
    # Scale the velocity values by 1000 to match the expected format
    velocity_x = int(float(velocity_x) * 1000)
    velocity_y = int(float(velocity_y) * 1000)
    angular_z = int(float(angular_z) * 1000)
    stepperx = int((stepperx+AXIS_X_LIMIT)/(2*AXIS_X_LIMIT)*Stepper_X_LIMIT)
    steppery = int((steppery+AXIS_Y_LIMIT)/(2*AXIS_Y_LIMIT)*Stepper_Y_LIMIT)
    print([stepperx,steppery])
    compressor1 = int(compressor1)
    compressor2 = int(compressor2)
    print(compressor1)
    print(compressor2)
    command = [
        0x7B,  # FRAME_HEADER
        0x00,
        0x00,
        velocity_x >> 8 & 0xFF,  # High byte of velocity_x
        velocity_x & 0xFF,  # Low byte of velocity_x
        velocity_y >> 8 & 0xFF,  # High byte of velocity_y
        velocity_y & 0xFF,  # Low byte of velocity_y
        angular_z >> 8 & 0xFF,  # High byte of angular_z
        angular_z & 0xFF,  # Low byte of angular_z
        steppery >> 8 & 0xFF,  # High byte of stepper1
        steppery & 0xFF,  # Low byte of stepper1
        stepperx >> 8 & 0xFF,  # High byte of stepper2
        stepperx & 0xFF,  # Low byte of stepper2
        compressor1 >> 8 & 0xFF,  # High byte of compressor1
        compressor1 & 0xFF,  # Low byte of compressor1
        compressor2 >> 8 & 0xFF,  # High byte of compressor2
        compressor2 & 0xFF,  # Low byte of compressor2
        0x00,  # Placeholder for checksum (to be calculated)
        0x7D  # FRAME_TAIL
    ]

    checksum = Check_Sum(command, 17, 0)  # Sum of bytes excluding FRAME_HEADER and checksum itself
    command[17] = checksum
    return command


def main(args=None):
    rclpy.init(args=None)
    serial_handler = SerialHandler()
    try :
        rclpy.spin(serial_handler)
    except KeyboardInterrupt:
        serial_handler.get_logger().info("Interrupted by user, shutting down")
        command_bytes = generate_command_bytes(0, 0, 0,-AXIS_X_LIMIT,-AXIS_Y_LIMIT, 0, 0)
        serial_handler.ser.write(bytes(command_bytes))
        serial_handler.ser.close()
        serial_handler.destroy_node()


if __name__ == '__main__':
    main()