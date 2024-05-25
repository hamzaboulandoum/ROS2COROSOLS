import time
from custom_interfaces.srv import Commands # type: ignore
from action_tutorials_interfaces.action import Position # type: ignore
from rclpy.action import ActionServer

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from nav_msgs.msg import Odometry
#from custom_interfaces.msg import Target

class SendVelocity(Node):
    def __init__(self):
        super().__init__('send_velocity')
        
        # Create a command client to communicate with serial
        self.command_cli = self.create_client(Commands, 'commands')
        while not self.command_cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        self.command_req = Commands.Request()  
        
        # Getting Data from Odom
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_listener, 10)
        self.odoData = Odometry()
        
        
    def odom_listener(self,msg):
        self.odoData = msg
        
    def calculate_velocities(self, x, y, r, a):
        
        #Calculate velocities
        self.command_req.vx = (x-self.odoData.pose.pose.position.x )
        self.command_req.vy = (y-self.odoData.pose.pose.position.x )
        self.command_req.vr = (r-self.odoData.pose.pose.position.z )
        self.command_req.airbrush = a 
        
        self.future = self.command_cli.call_async(self.command_req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=1)
        return self.future.result()
    
    
class VelocityCalculator(Node):

    def __init__(self,test):
        super().__init__('velocity_calculator')
        self._action_server = ActionServer(
            self,
            Position,
            'position_server',
            self.calculate_targets,
            )
        self.test = test 
        
                 
    
    def calculate_targets(self, goal_handle): 
        
        self.get_logger().info('Incoming request : x: %d y: %d r: %d a: %d' % (goal_handle.request.x, goal_handle.request.y, goal_handle.request.r, goal_handle.request.a)) 
        
        feedback_msg = Position.Feedback()
        feedback_msg.precision_log = []
        
        for i in range(1, 5):
            
            feedback_msg.precision_log.append(0.5)
            
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.precision_log))
            goal_handle.publish_feedback(feedback_msg)
            
            #gettinng feedback from serail server to know if command went successfully or not
            outcome = self.test.calculate_velocities( goal_handle.request.x, goal_handle.request.y, goal_handle.request.r, goal_handle.request.a)
            '''self.get_logger().info('Your Request : x = %d , y = %d , r = %d, a = %d was %s'
            % (int(self.command_req.vx),int(self.command_req.vy),int(self.command_req.vr),int(self.command_req.airbrush), str(outcome)))
            '''
            self.get_logger().info(f'Your Request{outcome}')
            
            time.sleep(1)
        
        
        # Sending response to user input to let him know that his request has finished.
        goal_handle.succeed()

        result = Position.Result()
        result.precision = feedback_msg.precision_log
        return result
    
    def calculate_velocities(self, x, y, r, a):
        
        #Calculate velocities
        self.command_req.vx = 2 * x
        self.command_req.vy = 2 * y
        self.command_req.vr = 2 * r
        self.command_req.airbrush = 2 * a    
        
        
        self.future = self.command_cli.call_async(self.command_req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=1)
        return self.future.result()
        
        
   
        
        


def main():

    rclpy.init()
    send_velocity = SendVelocity()
    velocity_calculator = VelocityCalculator(send_velocity)

    rclpy.spin(velocity_calculator)
    send_velocity.destroy_node()
    velocity_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    

    