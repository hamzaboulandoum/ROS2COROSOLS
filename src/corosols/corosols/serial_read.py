import time
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import SerialData,Command # type: ignore
import serial

serial_port = "/dev/ttyACM0" # Update with your serial port
baud_rate = 115200

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.ser = serial.Serial(serial_port, baud_rate)
        self.publisher_ = self.create_publisher(SerialData, 'topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Command,
            'commands',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        
        command_bytes = generate_command_bytes(msg.vx, msg.vy,msg.vr,msg.airbrush)
        for byte in command_bytes:
            self.ser.write(bytes([byte]))
            time.sleep(0.001)
        self.get_logger().info(f"{msg}") 
            
    def timer_callback(self):
        msg = SerialData()
        
        
        
        received_data = self.ser.read(24)
        Frame_Header = received_data[0]  # Frame header (byte)
        Flag_Stop = received_data[1]  # Stop flag (byte)

        X_speed = (received_data[2] << 8) | received_data[3]  # X-axis speed (16-bit integer)
        Y_speed = (received_data[4] << 8) | received_data[5]  # Y-axis speed (16-bit integer)
        Z_speed = (received_data[6] << 8) | received_data[7]  # Z-axis speed (16-bit integer)

        X_accel = (received_data[8] << 8) | received_data[9]   # X-axis acceleration (16-bit integer)
        Y_accel = (received_data[10] << 8) | received_data[11]  # Y-axis acceleration (16-bit integer)
        Z_accel = (received_data[12] << 8) | received_data[13]  # Z-axis acceleration (16-bit integer)

        X_gyro = (received_data[14] << 8) | received_data[15]  # X-axis gyroscope (16-bit integer)
        Y_gyro = (received_data[16] << 8) | received_data[17]  # Y-axis gyroscope (16-bit integer)
        Z_gyro = (received_data[18] << 8) | received_data[19]  # Z-axis gyroscope (16-bit integer)

        Power_Voltage = (received_data[20] << 8) | received_data[21] # Battery voltage (16-bit integer)

 
                
        Checksum = received_data[22]  # Checksum (byte)
        Frame_Tail = received_data[23]  # Frame tail (byte)
        #--------------------------
        if Check_Sum(received_data,22, 1)==Checksum:
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
            
            '''self.get_logger().info(f"""
                X_speed: {msg.x_speed}
                Y_speed: { msg.y_speed}
                Z_speed: { msg.z_speed}
                X_acceleration: { msg.x_accel}
                Y_acceleration: { msg.y_accel}
                Z_acceleration: { msg.z_accel}
                X_gyroscope: {  msg.x_gyro}
                Y_gyroscope: {  msg.y_gyro}
                Z_gyroscope: {  msg.z_gyro}            
                Battery Voltage: { msg.power_voltage}
                """)'''


            self.publisher_.publish(msg)
        
      
                
    


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

def generate_command_bytes(x, y, z, a):
    # Scale the velocity values by 1000 to match the expected format in the C++ code
    velocity_x = int(float(x) * 1000)
    velocity_y = int(float(y) * 1000)
    angular_z = int(float(z) * 1000)
    airbrush = int(a)
    if (airbrush == 1):
        airbrush = 1000
        print(airbrush)
        
    else: 
        airbrush = 2000
        print(airbrush)
    # Create the command bytes with header, reserved bytes, velocity components, checksum, and tail
    command = [
        0x7B,  # FRAME_HEADER
        0x00,  # Reserved
        0x00,  # Reserved
        velocity_x >> 8 & 0xFF,  # High byte of velocity_x
        velocity_x & 0xFF,       # Low byte of velocity_x
        velocity_y >> 8 & 0xFF,  # High byte of velocity_y
        velocity_y & 0xFF,       # Low byte of velocity_y
        angular_z >> 8 & 0xFF,   # High byte of angular_z
        angular_z & 0xFF,  
        airbrush >> 8 & 0xFF,   # High byte of angular_z
        airbrush & 0xFF,
        0x00,  # Placeholder for checksum (to be calculated)
        0x7D   # FRAME_TAIL
    ]
    
    checksum = Check_Sum(command,11, 0)  # Sum of bytes excluding FRAME_HEADER and checksum itself
    command[11] = checksum
    
    return command
def main(args=None):
    global minimal_publisher
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()