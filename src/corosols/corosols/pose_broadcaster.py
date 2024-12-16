import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from custom_interfaces.msg import SerialData 

from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from std_msgs.msg import Header,String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np
from custom_interfaces.msg import ImuData
import socket

# Global transaction ID for GeoCOM requests
GTrId = 0

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class tf2_broadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')
        self.ip = '10.27.212.64'
        self.port = 1212
        self.station_socket = None
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(ImuData,'imu_data',self.imu_callback,10)
        timer_period = 0.05  # seconds        
        self.timer = self.create_timer(timer_period, self.station_callback)

        self.params = self.create_subscription(String,'robot_params',self.params_listener,10)
        self.parameters_publisher = self.create_publisher(String, 'robot_params', 10)
        self.Odom = Odometry()
        self.x=0
        self.y=0
        self.z=0

        self.roll_offset = 0
        self.pitch_offset = 0
        self.heading_offset = 0

        self.x_accel = 0
        self.y_accel = 0
        self.z_accel = 0

        self.timestamp = self.get_clock().now().to_msg()
        self.imu_timestamp = None

    def params_listener(self,msg):
        data = msg.data
        params = data.split(';')
        if params[0] == 'ip':
            self.ip = params[1]
        elif params[0] == 'port':
            self.port = int(params[1])
        elif params[0] == 'Connect_to_station':
            self.connect_to_station()
        elif params[0] == 'angles_offset':
            if params[1] =='1':
                self.roll_offset = self.Odom.pose.pose.orientation.x 
                self.pitch_offset = self.Odom.pose.pose.orientation.y
                self.heading_offset = self.Odom.pose.pose.orientation.z
            else :
                self.roll_offset = 0
                self.pitch_offset = 0
                self.heading_offset = 0
    def station_callback(self):
        if self.station_socket:
            response  = self.getposition()
            if response:
                x, y, z = response
            else :
                self.parameters_publisher.publish(String(data='Station_error;E1'))
            self.x = x
            self.y = y
            self.z = z
            self.timestamp = self.get_clock().now().to_msg()
        
            self.Odom.header = Header()
            self.Odom.header.stamp = self.get_clock().now().to_msg()
            self.Odom.header.frame_id = 'odom'
            
            
            # we can change the postion with the postion from the marvel mind device 
            self.Odom.pose.pose.position.x = self.x 
            self.Odom.pose.pose.position.y = self.y
            self.Odom.pose.pose.position.z = self.z

            self.odom_publisher.publish(self.Odom)
            
        else :
            self.parameters_publisher.publish(String(data='Station_status;0'))
    
    def imu_callback(self, msg):
        
        if not self.imu_timestamp :
            self.imu_timestamp = msg.timestamp

        delta_time = float(msg.timestamp - self.imu_timestamp)*10**-6
        
        self.imu_timestamp = msg.timestamp


        self.x_accel, self.y_accel, self.z_accel = self.transform_acceleration(msg.accelerometer_x, msg.accelerometer_y, msg.accelerometer_z, msg.roll, msg.pitch, msg.heading)
        
        self.Odom.twist.twist.linear.x = self.x_accel*(delta_time) #speed X
        self.Odom.twist.twist.linear.y = self.y_accel*(delta_time) #speed Y
        self.Odom.twist.twist.linear.z = self.z_accel*(delta_time)
        
        self.Odom.pose.pose.orientation.x = msg.roll - self.roll_offset
        self.Odom.pose.pose.orientation.y = msg.pitch - self.pitch_offset
        self.Odom.pose.pose.orientation.z = msg.heading - self.heading_offset

        
        
        self.odom_publisher.publish(self.Odom)

    def transform_acceleration(self, raw_x, raw_y, raw_z, roll, pitch, heading):
        """
        Transforms raw acceleration values to actual acceleration values using roll, pitch, and heading.

        Args:
            raw_x (float): Raw acceleration along the X-axis in the body frame.
            raw_y (float): Raw acceleration along the Y-axis in the body frame.
            raw_z (float): Raw acceleration along the Z-axis in the body frame.
            roll (float): Roll angle in radians.
            pitch (float): Pitch angle in radians.
            heading (float): Heading (yaw) angle in radians.

        Returns:
            (tuple): Transformed acceleration values in the global frame (x, y, z).
        """
        # Rotation matrices
        R_roll = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])

        R_pitch = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        R_heading = np.array([
            [math.cos(heading), -math.sin(heading), 0],
            [math.sin(heading), math.cos(heading), 0],
            [0, 0, 1]
        ])

        # Combined rotation matrix (Z * Y * X order)
        R = R_heading @ R_pitch @ R_roll

        # Raw acceleration vector in the body frame
        raw_accel = np.array([raw_x, raw_y, raw_z])

        # Transform to global frame
        global_accel = R @ raw_accel

        return global_accel[0], global_accel[1], global_accel[2]
    def connect_to_station(self, timeout=5):
        """
        Establish a TCP/IP connection to the Leica TS13 station.

        Parameters:
            ip (str): IP address of the station.
            port (int): Port number.
            timeout (int): Timeout for socket operations in seconds.

        Returns:
            socket.socket: Socket connection object.
        """
        self.station_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.station_socket.settimeout(timeout)
        self.station_socket.connect((self.ip, self.port))
        if self.station_socket:
            self.parameters_publisher.publish(String(data='Station_status;1'))

    def TMC_QuickDist(self):
        """
        Perform a quick distance measurement.

        Parameters:
            station_socket (socket.socket): The TCP/IP socket connection to the station.

        Returns:
            str: Measurement result or error message.
        """
        response = self.send_command_to_station( 2117)
        if response :
            distance_result = response.split(",")
            alpha = float(distance_result[3])
            omega = float(distance_result[4])
            distance = float(distance_result[5])
            return alpha, omega, distance
        else :
            self.parameters_publisher.publish(String(data='Station_error;E1'))
    # Function to send a command and receive a response
    def send_command_to_station(self, command, args=None):
        """
        Send a GeoCOM command to the station and receive its response.

        Parameters:
            station_socket (socket.socket): The TCP/IP socket connection to the station.
            command (int): GeoCOM command code.
            args (list, optional): Arguments for the command.

        Returns:
            str: Response from the station.
        """
        try:
            request = CreateRequest(command, args)
            self.station_socket.sendall(request.encode('utf-8'))

            # Wait and receive response
            response = self.station_socket.recv(1024).decode('utf-8').strip()
            return response
        except Exception as e:
            return None


    # Example GeoCOM functions


    def BAP_SearchTarget(self):
        """
        Search for a prism target.

        Parameters:
            station_socket (socket.socket): The TCP/IP socket connection to the station.

        Returns:
            str: Search result or error message.
        """
        response = self.send_command_to_station(17020, [0])
        return response

    def BAP_MeasDistanceAngle(self, mode=6):
        """
        Measure distance and angle.

        Parameters:
            station_socket (socket.socket): The TCP/IP socket connection to the station.
            mode (int): Measurement mode (default is 6).

        Returns:
            tuple: (error, response_code, coordinates)
        """
        response = self.send_command_to_station(17017)
        coord = []
        error = None

        if response:
            try:
                parameters = response.split(',')[2:]
                if len(parameters) >= 4:
                    coord = [
                        float(parameters[0]),
                        float(parameters[1]),
                        float(parameters[2]),
                        int(parameters[3])
                    ]
                    print(f"Got data successfully: {coord}")
            except Exception as e:
                error = str(e)

        return error, response, coord
    
    def getposition(self):
        response = self.TMC_QuickDist()
        if response:
            theta, phi , distance = response
            return spherical_to_cartesian(distance, theta, phi)
        else :
            self.parameters_publisher.publish(String(data='Station_error;E1'))
            return None
    

def main(args=None):
    rclpy.init(args=args)

    tf2_broadcaster_corosols = tf2_broadcaster()

    try:
        rclpy.spin(tf2_broadcaster_corosols)
    except KeyboardInterrupt:
        tf2_broadcaster_corosols.destroy_node()


if __name__ == '__main__':
    main()

def spherical_to_cartesian(r, theta, phi):
        """
        Convert spherical coordinates to Cartesian coordinates.
        
        Parameters:
        r (float): Radius
        theta (float): Azimuthal angle in radians
        phi (float): Polar angle in radians
        
        Returns:
        tuple: (x, y, z) Cartesian coordinates
        """
        x = r * math.sin(phi) * math.cos(theta)
        y = r * math.sin(phi) * math.sin(theta)
        z = r * math.cos(phi)
        return x, y, z

# Function to create GeoCOM request
def CreateRequest(cmd, args=None):
    """
    Create an ASCII request based on a command code and corresponding arguments.

    Parameters:
        cmd (int): Function code to send to the station.
        args (list, optional): List of arguments for the command.

    Returns:
        str: Formatted GeoCOM request string.
    """
    global GTrId
    request = '\n%R1Q,' + str(cmd) + ',' + str(GTrId) + ':'
    GTrId += 1
    if GTrId == 8:
        GTrId = 0

    if args:
        request += ','.join(map(str, args))
    return request + '\r\n'

