import sys
import threading
import tkinter as tk
from tkinter import filedialog
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_tutorials_interfaces.action import Position

class UserInput(Node):
    def __init__(self):
        super().__init__('user_input')
        self._action_client = ActionClient(self, Position, 'position_server')
        self.gcode_lines = []
        self.waiting =False

    def send_goal(self, x, y, z, a):
        goal_msg = Position.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.r = z
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
        self.waiting = False

    def process_gcode_file(self, file_path):
        self.get_logger().info(f'Processing G-code file: {file_path}')
        with open(file_path, 'r') as file:
            self.gcode_lines = file.readlines()

    def start_sending_gcode(self):
        self.get_logger().info('Starting to send G-code lines')
        for line in self.gcode_lines:
            # Parse the G-code line to extract coordinates
            parts = line.strip().split()
            x, y, z, a = 0.0, 0.0, 0.0, 0  # Default values
            for part in parts:
                if part.startswith('X'):
                    x = float(part[1:])
                elif part.startswith('Y'):
                    y = float(part[1:])
                elif part.startswith('Z'):
                    z = float(part[1:])
                elif part.startswith('F'):
                    a = int(part[1:])
            self.get_logger().info(f'x {x} \t y{y}\t z{z}')
            # Send the coordinates to the action server
            self.send_goal(x, y, z, a)
            self.waiting=True
            while self.waiting:
                rclpy.spin_once(self)

class GCodeApp:
    def __init__(self, root, node):
        self.node = node
        self.root = root
        self.root.title("G-code Sender")
        self.upload_button = tk.Button(root, text="Upload G-code File", command=self.upload_file)
        self.upload_button.pack(pady=10)
        self.start_button = tk.Button(root, text="Start", command=self.start_sending)
        self.start_button.pack(pady=10)
        self.file_path = None

    def upload_file(self):
        self.file_path = filedialog.askopenfilename(filetypes=[("G-code files", "*.gcode"), ("All files", "*.*")])
        if self.file_path:
            print(f'File uploaded: {self.file_path}')  # Debugging line
            self.node.process_gcode_file(self.file_path)

    def start_sending(self):
        if self.file_path:
            print('Starting to send G-code')  # Debugging line
            self.node.start_sending_gcode()

def ros_spin(node):
    rclpy.spin_once(node, timeout_sec=0.1)

def main():
    rclpy.init()
    user_input_node = UserInput()
    
    # Start ROS 2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(user_input_node,))
    ros_thread.start()
    
    # Create and start the Tkinter main loop
    root = tk.Tk()
    app = GCodeApp(root, user_input_node)
    root.mainloop()

    # Ensure ROS 2 thread is properly shut down
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
