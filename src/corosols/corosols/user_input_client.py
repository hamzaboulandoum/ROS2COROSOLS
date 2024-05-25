import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

#from custom_interfaces.srv import Input # type: ignore
from action_tutorials_interfaces.action import Position # type: ignore



class UserInput(Node):

    def __init__(self):
        super().__init__('user_input')
        
        #self.cli = self.create_client(Input, 'input')
        """while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Input.Request()"""

        self._action_client = ActionClient(self, Position, 'position_server')
    
        
    def send_goal(self, x, y, r, a):
        goal_msg = Position.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.r = r
        goal_msg.a = a

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        
     

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.precision))
        rclpy.shutdown()
     
     
     
    
    """def send_request(self, x, y, r, a):
        self.req.x = x
        self.req.y = y
        self.req.r = r
        self.req.a = a
                    
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()"""


def main():
    rclpy.init()
    
    user_input = UserInput()
    
    user_input.send_goal(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]))
    rclpy.spin(user_input)
    


if __name__ == '__main__':
    main()