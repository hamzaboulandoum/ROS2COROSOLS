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
import math
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
        #self.subscription = self.create_subscription(SerialData,'robot_data',self.listener_callback,10)
        timer_period = 0.05  # seconds        
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.params = self.create_subscription(String,'robot_params',self.params_listener,10)
        self.parameters_publisher = self.create_publisher(String, 'robot_params', 10)

        self.x=0
        self.y=0
        self.z=0
        self.timestamp = self.get_clock().now().to_msg()
    def params_listener(self,msg):
        data = msg.data
        params = data.split(';')
        if params[0] == 'ip':
            self.ip = params[1]
        elif params[0] == 'port':
            self.port = int(params[1])
        elif params[0] == 'Connect_to_station':
            self.connect_to_station()
    def timer_callback(self):
        Odom = Odometry()
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
        
            Odom.header = Header()
            Odom.header.stamp = self.get_clock().now().to_msg()
            Odom.header.frame_id = 'odom'
            
            
            odom_quat = quaternion_from_euler(0, 0, self.z)
            
            # we can change the postion with the postion from the marvel mind device 
            Odom.pose.pose.position.x = self.x 
            Odom.pose.pose.position.y = self.y
            Odom.pose.pose.position.z = self.z
            Odom.pose.pose.orientation.x = odom_quat[0]
            Odom.pose.pose.orientation.y = odom_quat[1]
            Odom.pose.pose.orientation.z = odom_quat[2]
            Odom.pose.pose.orientation.w = odom_quat[3]
            
            Odom.child_frame_id = 'base_link'
            Odom.twist.twist = Twist()
            Odom.twist.twist.linear.x = 0.0 #speed X 
            Odom.twist.twist.linear.y = 0.0 #speed X 
            Odom.twist.twist.angular.z = 0.0 #angular speed
            
            self.odom_publisher.publish(Odom)
        else :
            self.parameters_publisher.publish(String(data='Station_status;0'))
    
    def listener_callback(self, msg):
        
        Odom = Odometry()
        
        delta_time = float(self.get_clock().now().to_msg().sec - self.timestamp.sec)
        delta_time +=  (self.get_clock().now().to_msg().nanosec - self.timestamp.nanosec)*10**-9
        
            
        self.x += -msg.y_speed*(delta_time)
        self.y += msg.x_speed*(delta_time)
        self.z += msg.z_speed*(delta_time)
        
        
        self.timestamp = self.get_clock().now().to_msg()
        
        Odom.header = Header()
        Odom.header.stamp = self.get_clock().now().to_msg()
        Odom.header.frame_id = 'odom'
        
        
        odom_quat = quaternion_from_euler(0, 0, self.z)
        
        # we can change the postion with the postion from the marvel mind device 
        Odom.pose.pose.position.x = self.x 
        Odom.pose.pose.position.y = self.y
        Odom.pose.pose.position.z = self.z
        Odom.pose.pose.orientation.x = odom_quat[0]
        Odom.pose.pose.orientation.y = odom_quat[1]
        Odom.pose.pose.orientation.z = odom_quat[2]
        Odom.pose.pose.orientation.w = odom_quat[3]
        
        Odom.child_frame_id = 'base_link'
        Odom.twist.twist = Twist()
        Odom.twist.twist.linear.x = -msg.y_speed
        Odom.twist.twist.linear.y = msg.x_speed
        Odom.twist.twist.angular.z = msg.z_gyro
        
        self.odom_publisher.publish(Odom)
        #self.get_logger().info(f'Publishing: \n x= {Odom.pose.pose.position.x}  vx = {msg.x_speed} \n y= {Odom.pose.pose.position.y}  vy = {msg.y_speed} \n z= {Odom.pose.pose.position.z}  vz = {msg.z_speed}')
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

