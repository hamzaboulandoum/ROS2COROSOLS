import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from custom_interfaces.msg import Target , Command


class MinimalSubscriber(Node):

    def __init__(self):
        
        super().__init__('minimal_subscriber')
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_listener,
            10)
        self.odom_subscription  # prevent unused variable warning
        
        self.target_subscription = self.create_subscription(
            Target,
            'target',
            self.target_listener,
            10)
        
        self.target_subscription  # prevent unused variable warning
        self.odom= Odometry()
        self.target= Target()

        self.publisher_ = self.create_publisher(Command, 'commands', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.command_callback)
        
    def command_callback(self): 
        msg = Command()
        msg.vx = int((self.target.x-self.odom.pose.pose.position.x)/50)
        msg.vy = int((self.target.y-self.odom.pose.pose.position.y)/50)
        msg.vr = int((self.target.r-self.odom.pose.pose.position.z)/50)
        msg.airbrush = self.target.airbrush
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

    def odom_listener(self, msg):
        self.odom = msg


    def target_listener(self, msg):
        self.target = msg

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()