import time
from custom_interfaces.srv import Commands # type: ignore
from action_tutorials_interfaces.action import Position # type: ignore
from rclpy.action import ActionServer
import serial
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from nav_msgs.msg import Odometry
#from custom_interfaces.msg import Target

class SendVelocity2(Node):
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
    
    
        
    def calculate_velocities(self, x, y, r, a,prec,kp):
        def sign(x):
            if x > 0:
                return 1
            elif x < 0:
                return -1
            else:
                return 0
        #Calculate velocities
        if abs(x-self.odoData.pose.pose.position.x )>prec:
            self.command_req.vx = sign(x-self.odoData.pose.pose.position.x )*0.2
            if abs(x-self.odoData.pose.pose.position.x ) < 0.2:
                self.command_req.vx = sign(x-self.odoData.pose.pose.position.x )*0.02
        else : 
            self.command_req.vx=0.0
            
        if abs(y-self.odoData.pose.pose.position.y )>prec:
            self.command_req.vy = sign(y-self.odoData.pose.pose.position.y)*0.2
            if abs(y-self.odoData.pose.pose.position.y ) < 0.2:
                self.command_req.vy = sign(y-self.odoData.pose.pose.position.y)*0.02
        else : 
            self.command_req.vy=0.0
            
        if abs(r-self.odoData.pose.pose.position.z )>prec:
            self.command_req.vr = sign(r-self.odoData.pose.pose.position.z )*0.2
            if abs(r-self.odoData.pose.pose.position.z ) < 0.2:
                self.command_req.vr = sign(r-self.odoData.pose.pose.position.z )*0.02
        else : 
            self.command_req.vr=0.0
            
        self.command_req.airbrush = a 
        
        self.future = self.command_cli.call_async(self.command_req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=1)
        return self.future.result()
    def stop(self):
        self.command_req.vx =0.0
        self.command_req.vy =0.0
        self.command_req.vr = 0.0
        self.command_req.airbrush = 0
        
        self.future = self.command_cli.call_async(self.command_req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=1)
        return self.future.result()
    
    
class VelocityCalculator2(Node):

    def __init__(self,test):
        super().__init__('velocity_calculator')
        self._action_server = ActionServer(
            self,
            Position,
            'position_server',
            self.calculate_targets,
            )
        self.test = test 
        self.prec =0.01
        self.kp = 0.8
    
    def calculate_targets(self, goal_handle): 
        
        self.get_logger().info('Incoming request : x: %d y: %d r: %d a: %d' % (goal_handle.request.x, goal_handle.request.y, goal_handle.request.r, goal_handle.request.a)) 
        
        feedback_msg = Position.Feedback()
        feedback_msg.precision_log = []
        
        while abs(goal_handle.request.x-self.test.odoData.pose.pose.position.x )>self.prec or abs(goal_handle.request.y-self.test.odoData.pose.pose.position.y )>self.prec or abs(goal_handle.request.r-self.test.odoData.pose.pose.position.z )>self.prec:
            
            feedback_msg.precision_log.append(abs(goal_handle.request.x-self.test.odoData.pose.pose.position.x ) + abs(goal_handle.request.y-self.test.odoData.pose.pose.position.y ) + abs(goal_handle.request.r-self.test.odoData.pose.pose.position.z ))
            
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.precision_log))
            goal_handle.publish_feedback(feedback_msg)
            
            #gettinng feedback from serail server to know if command went successfully or not
            outcome = self.test.calculate_velocities( goal_handle.request.x, goal_handle.request.y, goal_handle.request.r, goal_handle.request.a,self.prec,self.kp )
            '''self.get_logger().info('Your Request : x = %d , y = %d , r = %d, a = %d was %s'
            % (int(self.command_req.vx),int(self.command_req.vy),int(self.command_req.vr),int(self.command_req.airbrush), str(outcome)))
            '''
            self.get_logger().info(f'Your Request{outcome}')
            
            time.sleep(0.2)
        self.test.stop( )
        
        # Sending response to user input to let him know that his request has finished.
        goal_handle.succeed()

        result = Position.Result()
        result.precision = feedback_msg.precision_log
        return result
    
        
        
   
        
        


def main():

    rclpy.init()
    send_velocity = SendVelocity2()
    velocity_calculator = VelocityCalculator2(send_velocity)

    rclpy.spin(velocity_calculator)
    send_velocity.destroy_node()
    velocity_calculator.destroy_node()
    serial.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    
    

    