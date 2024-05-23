import sys
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import Target
class MinimalPublisher(Node):

    def __init__(self,x,y,r,a):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Target, 'target', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = x
        self.y = y
        self.r = r
        self.a = a
        self.get_logger().info(f"1111 {x}") 
        self.timer_callback()
        

    def timer_callback(self):
        msg = Target()
        msg.x = self.x
        msg.y = self.y
        msg.r = self.r
        msg.airbrush = self.a
        self.publisher_.publish(msg)
        self.get_logger().info(f"{msg}") 
    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalPublisher(int(sys.argv[1]),int(sys.argv[2]),int(sys.argv[3]),int(sys.argv[4]))

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()