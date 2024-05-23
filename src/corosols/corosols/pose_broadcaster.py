import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from custom_interfaces.msg import SerialData 

from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import numpy as np


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
        
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)
        self.subscription = self.create_subscription(
            SerialData,
            'topic',
            self.listener_callback,
            10)
        self.x=0
        self.y=0
        self.z=0
        self.timestamp = self.get_clock().now().to_msg()






    def listener_callback(self, msg):
        
        Odom = Odometry()
        
        delta_time = float(self.get_clock().now().to_msg().sec - self.timestamp.sec)
        delta_time +=  (self.get_clock().now().to_msg().nanosec - self.timestamp.nanosec)*10**-9
        
        self.x += msg.x_speed*(delta_time)
        self.y += msg.y_speed*(delta_time)
        self.z += msg.z_speed*(delta_time)
        
        
        self.timestamp = self.get_clock().now().to_msg()
        
        Odom.header = Header()
        Odom.header.stamp = self.get_clock().now().to_msg()
        Odom.header.frame_id = 'odom'
        
        
        odom_quat = quaternion_from_euler(0, 0, self.z)
        
        '''t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]
        self.tf_static_broadcaster.sendTransform(t)    '''
        
        Odom.pose.pose.position.x = self.x
        Odom.pose.pose.position.y = self.y
        Odom.pose.pose.position.z = self.z
        Odom.pose.pose.orientation.x = odom_quat[0]
        Odom.pose.pose.orientation.y = odom_quat[1]
        Odom.pose.pose.orientation.z = odom_quat[2]
        Odom.pose.pose.orientation.w = odom_quat[3]
        
        Odom.child_frame_id = 'base_link'
        Odom.twist.twist = Twist()
        Odom.twist.twist.linear.x = msg.x_speed
        Odom.twist.twist.linear.y = msg.y_speed
        Odom.twist.twist.angular.z = msg.z_gyro
        
        self.odom_publisher.publish(Odom)
        self.get_logger().info(f'Publishing: {Odom}')
        
        self.get_logger().info(f'Publishing: {delta_time}')

def main(args=None):
    rclpy.init(args=args)

    tf2_broadcaster_corosols = tf2_broadcaster()

    rclpy.spin(tf2_broadcaster_corosols)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tf2_broadcaster_corosols.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()