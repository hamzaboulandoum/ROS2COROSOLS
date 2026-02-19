import threading
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from custom_interfaces.msg import SerialData 

from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from std_msgs.msg import Header, String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np
from custom_interfaces.msg import ImuData
from custom_interfaces.msg import SerialData 
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


class tf2_broadcaster(Node):

    def __init__(self):
        super().__init__('tf2_broadcaster')
        self.ip = '10.27.212.64'
        self.port = 1212
        self.station_socket = None
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.subscription = self.create_subscription(ImuData, 'imu_data', self.imu_callback, 10)
        #timer_period = 0.01  # seconds        
        #self.timer = self.create_timer(timer_period, self.station_callback)
        self.subscription2 = self.create_subscription(SerialData, 'robot_data', self.position_callback, 10)

        self.params = self.create_subscription(String, 'robot_params', self.params_listener, 10)
        self.parameters_publisher = self.create_publisher(String, 'robot_params', 10)
        self.Odom = Odometry()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        # Lock for thread-safe access to position data
        self.position_lock = threading.Lock()
        # Lock for thread-safe access to socket
        self.socket_lock = threading.Lock()
        
        # Flag to track if new station data is available
        self.new_station_data = False
        
        self.roll_offset = 0
        self.pitch_offset = 0
        self.heading_offset = 0

        self.x_accel = 0
        self.y_accel = 0
        self.z_accel = 0

        # Initialisation des variables de calibration manquantes
        self.is_accel_calibrated = False
        self.accel_calibration_samples = []
        self.calibration_count = 50
        self.accel_offset = np.array([0.0, 0.0, 0.0])
        self.accel_linear = np.array([0.0, 0.0, 0.0])

        self.timestamp = self.get_clock().now().to_msg()
        self.imu_timestamp = None
        self.t = time.time()
        self.error_is_sent = False
        self.station_freq = 0
        self.station_thread = threading.Thread(target=self.station_callback)
        self.station_thread.start()
        

    def params_listener(self, msg):
        data = msg.data
        params = data.split(';')
        if params[0] == 'ip':
            self.ip = params[1]
        elif params[0] == 'port':
            self.port = int(params[1])
        elif params[0] == 'Connect_to_station':
            self.connect_to_station()
        elif params[0] == 'angles_offset':
            if params[1] == '1':
                self.roll_offset = self.Odom.pose.pose.orientation.x 
                self.pitch_offset = self.Odom.pose.pose.orientation.y
                self.heading_offset = self.Odom.pose.pose.orientation.z
            else:
                self.roll_offset = 0
                self.pitch_offset = 0
                self.heading_offset = 0

    '''def station_callback(self):
        if self.station_socket:
            self.t = time.time()
            response = self.getposition()
            self.station_freq = 1/(time.time()-self.t)
            
            if response:
                if len(response) != 3:
                    return
                x, y, z = response

                # self.x = x
                # self.y = y
                # self.z = z
                self.timestamp = self.get_clock().now().to_msg()
                #self.get_logger().info(f'freq: {self.station_freq:.2f}Hz x: {self.x} y: {self.y} z: {self.z}')
                self.Odom.header = Header()
                self.Odom.header.stamp = self.get_clock().now().to_msg()
                self.Odom.header.frame_id = 'odom'
                
                # we can change the postion with the postion from the marvel mind device 
                self.Odom.pose.pose.position.x = x 
                self.Odom.pose.pose.position.y = y
                self.Odom.pose.pose.position.z = z 
        else:
            self.parameters_publisher.publish(String(data='Station_status;0'))
        self.odom_publisher.publish(self.Odom)'''
    def station_callback(self):
        last_status_publish_time = 0
        status_publish_interval = 1.0  # Only publish status every 1 second
        
        while True:
            # Thread-safe check and use of socket
            with self.socket_lock:
                socket_available = self.station_socket is not None
            
            if socket_available:
                self.t = time.time()
                response = self.getposition()
                if time.time() - self.t > 0:
                    self.station_freq = 1/(time.time()-self.t)
                
                if response:
                    if len(response) != 3:
                        continue
                    x, y, z = response

                    # Thread-safe update of position
                    with self.position_lock:
                        self.x = x
                        self.y = y
                        self.z = z
                        self.timestamp = self.get_clock().now().to_msg()
                        self.new_station_data = True  # Mark that new data is available
                else:
                    # Rate-limit status publishing
                    current_time = time.time()
                    if current_time - last_status_publish_time > status_publish_interval:
                        self.parameters_publisher.publish(String(data='Station_status;0'))
                        last_status_publish_time = current_time
            else:
                # Rate-limit status publishing when no socket
                current_time = time.time()
                if current_time - last_status_publish_time > status_publish_interval:
                    self.parameters_publisher.publish(String(data='Station_status;0'))
                    last_status_publish_time = current_time
                # Add a small sleep to avoid busy-waiting when no socket
                time.sleep(0.1)
            #self.get_logger().info(f'freq: {self.station_freq:.2f}Hz x: {self.x} y: {self.y} z: {self.z}')
        
        
    def position_callback(self, msg):
        # Only publish if there's new station data
        
        #self.get_logger().info(f'freq: {self.station_freq:.2f}Hz x: {x} y: {y} z: {z}')
        self.Odom.header = Header()
        self.Odom.header.stamp = self.get_clock().now().to_msg()
        self.Odom.header.frame_id = 'odom'
        #self.x += msg.x_speed
        #self.y += msg.y_speed
        #self.z += msg.z_speed
        # Update position from station data
        self.Odom.pose.pose.position.x = self.x
        self.Odom.pose.pose.position.y = self.y 
        self.Odom.pose.pose.position.z = self.z
        
        self.odom_publisher.publish(self.Odom)
    def imu_callback(self, msg):
        
        if not self.imu_timestamp:
            self.imu_timestamp = msg.timestamp

        delta_time = float(msg.timestamp - self.imu_timestamp)*10**-6
        
        self.imu_timestamp = msg.timestamp

        accel_result = self.transform_acceleration(msg.accelerometer_x, msg.accelerometer_y, msg.accelerometer_z, msg.roll, msg.pitch, msg.heading)
        self.x += self.Odom.twist.twist.linear.x * delta_time + 0.5 * self.x_accel * delta_time**2
        self.y += self.Odom.twist.twist.linear.y * delta_time + 0.5 * self.y_accel * delta_time**2
        self.z += self.Odom.twist.twist.linear.z * delta_time + 0.5 * self.z_accel * delta_time**2
        #self.get_logger().info(f"IMU Data: x={self.x:.2f}, y={self.y:.2f}, z={self.z:.2f}, ")
        # V�rifier si transform_acceleration a retourn� des valeurs
        if accel_result is not None:
            self.x_accel, self.y_accel, self.z_accel = accel_result
            
            self.Odom.twist.twist.linear.x = self.x_accel*(delta_time)  # speed X
            self.Odom.twist.twist.linear.y = self.y_accel*(delta_time)  # speed Y
            self.Odom.twist.twist.linear.z = self.z_accel*(delta_time)
        
        self.Odom.pose.pose.orientation.x = msg.roll - self.roll_offset
        self.Odom.pose.pose.orientation.y = msg.pitch - self.pitch_offset
        self.Odom.pose.pose.orientation.z = msg.heading - self.heading_offset

    def calibrate_accel(self, accel_raw):
        """Calibre l'acc�l�rom�tre avec les premi�res mesures"""
        if not self.is_accel_calibrated:
            self.accel_calibration_samples.append(np.array(accel_raw))
            
            if len(self.accel_calibration_samples) >= self.calibration_count:
                self.accel_offset = np.mean(self.accel_calibration_samples, axis=0)
                self.is_accel_calibrated = True
                self.accel_offset[2] += 9.63  # Ajustement pour la gravit�
                print(f"Calibration Accel termin�e. Offset: X={self.accel_offset[0]:.3f}, "
                      f"Y={self.accel_offset[1]:.3f}, Z={self.accel_offset[2]:.3f} m/s�")
                self.accel_calibration_samples.clear()
                return True
            else:
                remaining = self.calibration_count - len(self.accel_calibration_samples)
                if len(self.accel_calibration_samples) % 5 == 0:
                    print(f"Calibration Accel en cours... {remaining} �chantillons restants")
                return False
        return True

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
        accel = np.array([raw_x, raw_y, raw_z])
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        gravity_vector = np.array([9.73 * sin_pitch, -9.78 * cos_pitch * sin_roll, -9.63 * cos_pitch * cos_roll])
        
        if not self.calibrate_accel(accel):  # Appel de la fonction renomm�e
            return None  # Retourner None au lieu de return sans valeur
    
        accel = accel - gravity_vector - self.accel_offset
        self.accel_linear = np.where(np.abs(accel) > 0.01, 
                                     accel, 0)

        return accel[0], accel[1], accel[2]

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
        try:
            new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            new_socket.settimeout(timeout)
            new_socket.connect((self.ip, self.port))
            
            # Thread-safe assignment of socket
            with self.socket_lock:
                self.station_socket = new_socket
            
            self.parameters_publisher.publish(String(data='Station_status;1'))
        except Exception as e:
            self.parameters_publisher.publish(String(data=f'Station_error;E2;{e}'))
            self.parameters_publisher.publish(String(data='Station_status;0'))
            with self.socket_lock:
                self.station_socket = None

    def TMC_QuickDist(self):
        """
        Perform a quick distance measurement.

        Parameters:
            station_socket (socket.socket): The TCP/IP socket connection to the station.

        Returns:
            str: Measurement result or error message.
        """
        response = self.send_command_to_station(2117)
        if response:
            try:
                distance_result = response.split(",")
                alpha = float(distance_result[3])
                omega = float(distance_result[4])
                distance = float(distance_result[5])
                return alpha, omega, distance
            except (IndexError, ValueError) as e:
                self.get_logger().error(f"Error parsing TMC_QuickDist response: {e}")
                return None
        else:
            self.parameters_publisher.publish(String(data='Station_error;E1'))
            return None

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
            
            # Thread-safe socket operation
            with self.socket_lock:
                if self.station_socket is None:
                    return None
                self.station_socket.sendall(request.encode('utf-8'))
                # Wait and receive response
                response = self.station_socket.recv(1024).decode('utf-8').strip()
            
            return response
        except Exception as e:
            if not self.error_is_sent:
                self.error_is_sent = True
                self.parameters_publisher.publish(String(data=f'Station_error;E3;{e}'))
            return None

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
            theta, phi, distance = response
            if self.error_is_sent and distance != 0.0:
                self.error_is_sent = False
            return spherical_to_cartesian(distance, theta, phi)
        else:
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