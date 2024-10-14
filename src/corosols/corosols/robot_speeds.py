import os
import sys
from tkinter import filedialog
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection
import time
import tkinter as tk
from tkinter import ttk
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from custom_interfaces.msg import SerialData
from custom_interfaces.srv import Commands
from action_tutorials_interfaces.action import Position
from rclpy.action import ActionServer
import serial


class Simulator:
    def __init__(self, target_points, printing, robot_speed, point_factor=1000, max_speed=0.1, min_speed=0.02):
        self.start_pos = robot_speed.getOdoData()
        self.target_points = target_points
        self.printing = printing
        self.robot_pos = self.start_pos.copy()
        self.airbrush_pos = self.start_pos.copy() + np.array([-AXIS_X_LIMIT,-AXIS_Y_LIMIT])
        self.current_target_index = 0
        self.robot_points = [self.start_pos.copy()]
        self.print_segments = []
        self.current_print_segment = []
        self.robot_velocities = []
        self.axis_velocities = []
        self.relative_airbrush_pos = np.array([-AXIS_X_LIMIT,-AXIS_Y_LIMIT])
        self.robot_speed = robot_speed
        self.robot_pid_controller = PIDController(kp=10, ki=0, kd=0)
        self.prev_time = time.time()
        
        # New parameters
        self.point_factor = point_factor
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.airbrush_acivation = True 

    def step(self, reset_target=False):
        if self.current_target_index >= len(self.target_points):
            self.robot_speed.command_req.vy = 0.0
            self.robot_speed.command_req.vx = 0.0
            self.robot_speed.command_req.airbrush = False
            self.robot_speed.command_req.stepperx = -AXIS_X_LIMIT
            self.robot_speed.command_req.steppery = -AXIS_Y_LIMIT
            self.robot_speed.send_speed()
            return False

        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
    
        current_target, _, _ = self.target_points[self.current_target_index]
        self.robot_pos = self.robot_speed.getOdoData()
        
        is_printing = self.printing[self.current_target_index]
        
        if self.current_target_index > 0:
            prev_target = self.target_points[self.current_target_index - 1][0]
        else:
            prev_target = self.start_pos
            
        if reset_target:
            current_target = np.array([0,0])
            is_printing = False
            prev_target = self.robot_pos
        
        # Linear movement
        self.airbrush_pos = self.robot_pos + np.array([self.robot_speed.robot_data.stepper_x,self.robot_speed.robot_data.stepper_y])
        
        self.relative_airbrush_pos = calculate_projection_point(prev_target, current_target, self.airbrush_pos, self.robot_pos, is_printing) 
        
        robot_velocity = calculate_robot_speed_vector(
            (self.airbrush_pos+self.robot_pos)/2, 
            prev_target,
            current_target, 
            self.max_speed,  # Use instance variable
            self.min_speed,  # Use instance variable
            dt,
            self.robot_pid_controller
        )
        self.robot_velocities.append(robot_velocity)
        
        if reset_target:
            ratio = np.clip(abs(self.robot_pos-current_target)/AXIS_Y_LIMIT/(self.max_speed/0.03),0,1)
        else:
            ratio = np.clip(abs(self.robot_pos-current_target)/AXIS_Y_LIMIT/(self.max_speed/0.03),0,1)*np.clip(abs(self.robot_pos-prev_target)/AXIS_Y_LIMIT/(self.max_speed/0.15),0.1/(self.max_speed/0.15),1)
        
        error = np.clip(np.linalg.norm(self.relative_airbrush_pos-np.array([self.robot_speed.robot_data.stepper_x,self.robot_speed.robot_data.stepper_y]))*20*(self.max_speed/0.15),1,100*(self.max_speed/0.15))
        self.robot_speed.command_req.vy = -float(robot_velocity[0]*ratio[0]/error)
        self.robot_speed.command_req.vx = float(robot_velocity[1]*ratio[1]/error)
        self.robot_speed.command_req.airbrush = is_printing
        
        self.robot_points.append(self.robot_pos.copy())

        if is_printing:
            if not self.current_print_segment:
                self.current_print_segment.append(self.airbrush_pos.copy())
            self.current_print_segment.append(self.airbrush_pos.copy())
        elif self.current_print_segment:
            self.print_segments.append(self.current_print_segment)
            self.current_print_segment = []

        # Check if we've reached the target.
        if np.linalg.norm(self.airbrush_pos - current_target) <= axis_precision:
            self.current_target_index += 1
        
        if reset_target and np.linalg.norm(self.robot_pos - current_target) <= 0.025:
            self.robot_speed.command_req.vy = 0.0
            self.robot_speed.command_req.vx = 0.0
            self.robot_speed.command_req.airbrush = False
            self.robot_speed.send_speed()
            return False
            
        self.robot_speed.command_req.stepperx = float(self.relative_airbrush_pos[0])
        self.robot_speed.command_req.steppery = float(self.relative_airbrush_pos[1])
        if not self.airbrush_acivation:
            self.robot_speed.command_req.airbrush = False
        self.robot_speed.send_speed()
                    
        return True
    
class RobotSpeed(Node):
    def __init__(self):
        super().__init__('robot_speed')
        
        # Create a command client to communicate with serial
        
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_listener, 1)
        self.robot_data_subscription  = self.create_subscription(
            SerialData, 'robot_data', self.robot_data_listener, 1)
        
        self.command_cli = self.create_client(Commands, 'commands')
        while not self.command_cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('service not available, waiting again...')
        self.command_req = Commands.Request()  
        
        # Getting Data from Odom
        
        self.odoData = Odometry()
        self.robot_data = SerialData()
        
        
    def odom_listener(self,msg):
        self.odoData = msg
        
    def robot_data_listener(self,msg):
        self.robot_data = msg
        
        
    def send_speed(self):
        try:    
            self.future = self.command_cli.call_async(self.command_req)
            rclpy.spin_until_future_complete(self, self.future,timeout_sec=1)
            return self.future.result()
        except KeyboardInterrupt:
            pass
    def getOdoData(self):
        return np.array([self.odoData.pose.pose.position.x,self.odoData.pose.pose.position.y])
        
def rclspin(node):
    try:
        rclpy.spin(node)
    except Exception:
        node.destroy_node()
        

class RobotControlUI(tk.Tk):
    def __init__(self, robot_speed):
        super().__init__()
        self.title("Robot Control UI")
        self.robot_speed = robot_speed
        self.simulator = None
        self.animation = None
        self.is_paused = True
        self.file_selected = True
        self.init_pos = False

        # Initialize parameters
        self.point_factor = tk.DoubleVar(value=1000)
        self.max_speed = tk.DoubleVar(value=0.1)
        self.min_speed = tk.DoubleVar(value=0.02)
        
        self.point_factor.trace_add('write', self.on_point_factor_change)
        self.max_speed.trace_add('write', self.on_speed_change)
        self.min_speed.trace_add('write', self.on_speed_change)
        
        self.configure(bg='#1e1e1e')
        self.geometry('1400x800')

        # Set full screen mode
        self.attributes('-fullscreen', True)
        self.bind('<Escape>', self.toggle_fullscreen)  # Bind Escape key to toggle fullscreen
        
        self.create_ui_elements()
        self.init_simulation()

        # Start updating robot data
        self.update_robot_data()
        
        # Set up the close handler
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Airbrush activation state
         # Airbrush activation state
        self.airbrush_start_stop = False 
    
    def toggle_fullscreen(self, event=None):
        self.attributes('-fullscreen', not self.attributes('-fullscreen'))
        
    def on_closing(self):
        os._exit(0)
        
    def on_point_factor_change(self, *args):
        self.init_simulation()

    def on_speed_change(self, *args):
        if self.min_speed:
            if self.min_speed.get() > 0:
                self.simulator.min_speed = self.min_speed.get()
        if self.max_speed:
            if self.max_speed.get() > 0:
                self.simulator.max_speed = self.max_speed.get()
        
    def create_ui_elements(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('.', background='#1e1e1e', foreground='#ffffff')
        style.configure('TButton', background='#0078d7', foreground='#ffffff')
        style.map('TButton', background=[('active', '#005fb3')])
        style.configure('TEntry', fieldbackground='#2d2d2d', foreground='#ffffff')
        style.configure('TLabel', background='#1e1e1e', foreground='#ffffff')

        # Create main frame
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create and pack the matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.fig.patch.set_facecolor('#1e1e1e')
        self.ax.set_facecolor('#2d2d2d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=main_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Create frame for controls and data display
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10)

        # File selection
        file_frame = ttk.Frame(control_frame)
        file_frame.pack(fill=tk.X, pady=10)
        self.filename_var = tk.StringVar(value=DEFAULT_FILENAME)
        ttk.Entry(file_frame, textvariable=self.filename_var, width=40).pack(side=tk.LEFT)
        ttk.Button(file_frame, text="Browse", command=self.browse_file).pack(side=tk.LEFT, padx=5)

        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.pack(fill=tk.X, pady=10)
        self.start_button = ttk.Button(button_frame, text="Start", command=self.toggle_simulation)
        self.start_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        
        button_frame2 = ttk.Frame(control_frame)
        button_frame2.pack(fill=tk.X, pady=10)
        
        self.airbrush_button = ttk.Button(button_frame2, text="Start Airbrush", command=self.start_stop_airbrush)
        self.airbrush_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

        self.airbrush_activation_button = ttk.Button(button_frame2, text="Deactivate Airbrush", command=self.airbrush_activation)
        self.airbrush_activation_button.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)

        # Parameter controls
        param_frame = ttk.Frame(control_frame)
        param_frame.pack(fill=tk.X, pady=10)
        self.create_parameter_control(param_frame, "Point Factor:", self.point_factor, 100, 5000)
        self.create_parameter_control(param_frame, "Max Speed (m/s):", self.max_speed, 0.01, 1.0)
        self.create_parameter_control(param_frame, "Min Speed (m/s):", self.min_speed, 0.01, 0.5)

        # Create labels for displaying values
        self.create_info_labels(control_frame)

    def airbrush_activation(self):
        if self.simulator.airbrush_acivation:
            self.airbrush_activation_button.config(text="Activate Airbrush")
        else:
            self.airbrush_activation_button.config(text="Deactivate Airbrush")
        self.simulator.airbrush_acivation = not self.simulator.airbrush_acivation
        # Optionally, send a command to the action server to change state
        # self.send_goal_for_airbrush_activation()

    def start_stop_airbrush(self):
        if self.airbrush_start_stop:
            self.airbrush_button.config(text="Stop Airbrush")
            self.robot_speed.command_req.airbrush = True
        else:
            self.airbrush_button.config(text="Start Airbrush")
            self.robot_speed.command_req.airbrush = False
        self.robot_speed.send_speed()
        self.airbrush_start_stop = not self.airbrush_start_stop

    def create_parameter_control(self, parent, label, variable, from_, to):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=5)
        ttk.Label(frame, text=label).pack(side=tk.LEFT)
        ttk.Scale(frame, from_=from_, to=to, variable=variable, orient=tk.HORIZONTAL).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
        ttk.Entry(frame, textvariable=variable, width=8).pack(side=tk.LEFT)

    def create_styled_label(self, parent, label_text, value_text="0.000", bg_color='#34495E', fg_color='#ECF0F1', value_fg_color='#2ECC71'):
        frame = tk.Frame(parent, bg=bg_color)
        frame.pack(fill=tk.X, padx=10, pady=5)
        
        label = tk.Label(frame, text=label_text, bg=bg_color, fg=fg_color, width=15, anchor='w')
        label.pack(side=tk.LEFT, padx=5)
        
        value_label = tk.Label(frame, text=value_text, bg=bg_color, fg=value_fg_color, width=10, anchor='e')
        value_label.pack(side=tk.LEFT, padx=5)
        
        return label, value_label

    def create_info_labels(self, parent):
        info_frame = ttk.Frame(parent)
        info_frame.pack(fill=tk.X, pady=10)
        
        _, self.robot_pos_label = self.create_styled_label(info_frame, "Robot Position:")
        _, self.airbrush_pos_label = self.create_styled_label(info_frame, "Airbrush Position:")
        _, self.progress_label = self.create_styled_label(info_frame, "Progress:")
        _, self.speed_label = self.create_styled_label(info_frame, "Robot Speed:")
        
        # Labels for additional robot data
        robot_data_frame = ttk.Frame(info_frame)
        robot_data_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(robot_data_frame, text="Robot Data:", font=('Helvetica', 12, 'bold')).pack(anchor='w', padx=5, pady=5)
        
        self.robot_data_labels = {}
        for data in ['x_speed', 'y_speed', 'z_speed', 'x_accel', 'y_accel', 'z_accel', 
                     'x_gyro', 'y_gyro', 'z_gyro', 'power_voltage', 'stepper_x', 'stepper_y']:
            _, self.robot_data_labels[data] = self.create_styled_label(robot_data_frame, f"{data.replace('_', ' ').title()}:")

    def browse_file(self):
        filename = filedialog.askopenfilename(filetypes=[("G-code files", "*.txt"), ("All files", "*.*")],initialdir=os.path.abspath('src/corosols/corosols/gcode'))
        if filename:
            self.filename_var.set(filename)
            self.file_selected = True
            self.init_simulation()

    def init_simulation(self):
        filename = self.filename_var.get()
        target_points, printing = parse_gcode(filename,self.point_factor.get())
        self.simulator = Simulator(target_points, printing, self.robot_speed,
                                   point_factor=self.point_factor.get(),
                                   max_speed=self.max_speed.get(),
                                   min_speed=self.min_speed.get())
        self.init_plot()

    def toggle_simulation(self):
        if not self.file_selected:
            return

        if self.is_paused:
            self.start_simulation()
        else:
            self.pause_simulation()

    def start_simulation(self):
        self.is_paused = False
        self.init_pos =False
        self.start_button.config(text="Pause")
        self.stop_button.config(state=tk.NORMAL)
        if self.animation is None:
            self.animation = FuncAnimation(self.fig, self.animate, interval=50, blit=True)
        self.canvas.draw()

    def pause_simulation(self):
        self.is_paused = True
        self.start_button.config(text="Resume")
        self.robot_speed.command_req.vx = 0.0
        self.robot_speed.command_req.vy = 0.0
        self.robot_speed.command_req.airbrush = False
        self.robot_speed.send_speed()

    def stop_simulation(self):
        if self.init_pos == False:
            self.start_button.config(text="Start")
            self.stop_button.config(state=tk.DISABLED)
            self.init_pos =True
            self.is_paused = False
        else :
            self.is_paused = True
            self.start_button.config(text="Start")
            self.stop_button.config(state=tk.DISABLED)
            self.robot_speed.command_req.vx = 0.0
            self.robot_speed.command_req.vy = 0.0
            self.robot_speed.command_req.airbrush = False
            self.robot_speed.send_speed()
            self.simulator.current_target_index = 0

    def init_plot(self):
        self.ax.clear()
        self.ax.set_facecolor('#2d2d2d')
        self.ax.set_aspect('equal', 'box')
        self.ax.set_title('G-code Mecanum Robot Simulation', color='white')
        self.ax.set_xlabel('X position (m)', color='white')
        self.ax.set_ylabel('Y position (m)', color='white')
        self.ax.tick_params(colors='white')
        self.ax.grid(True, color='#555555')
        if self.simulator:
            target_points_array = np.array([point[0] for point in self.simulator.target_points])
            self.ax.scatter(target_points_array[:, 0], target_points_array[:, 1], c='#00a8e8', s=2, alpha=0.5, label='Target Points')
            self.ax.scatter(self.simulator.robot_pos[0], self.simulator.robot_pos[1], c='#00ff00', s=100, label='Start Position')
        self.ax.legend(facecolor='#2d2d2d', edgecolor='#555555', labelcolor='white')
        self.ax.autoscale()

        # Initialize plot elements
        self.robot_circle = Circle(self.simulator.robot_pos, 0.02, fc='#ff0000', ec='#ff0000', label='Robot')
        self.airbrush_circle = Circle(self.simulator.airbrush_pos, 0.01, fc='#00a8e8', ec='#00a8e8', label='Airbrush')
        self.axis_rectangle = Rectangle(self.simulator.robot_pos - [AXIS_X_LIMIT, AXIS_Y_LIMIT], 
                                        2*AXIS_X_LIMIT, 2*AXIS_Y_LIMIT, fill=False, edgecolor='#ff8c00', linestyle='--')
        self.robot_path, = self.ax.plot([], [], color='#ff0000', linewidth=0.5, alpha=0.5, label='Robot Path')
        self.print_segments = LineCollection([], colors='#ffffff', linewidths=2, label='Print Path')

        self.ax.add_patch(self.robot_circle)
        self.ax.add_patch(self.airbrush_circle)
        self.ax.add_patch(self.axis_rectangle)
        self.ax.add_collection(self.print_segments)

        self.canvas.draw()

    def animate(self, frame):
        if self.simulator is None or self.is_paused:
            return self.robot_circle, self.airbrush_circle, self.axis_rectangle, self.robot_path, self.print_segments

        if self.simulator.step(self.init_pos):
            self.update_plot()
        else:
            self.stop_simulation()

        return self.robot_circle, self.airbrush_circle, self.axis_rectangle, self.robot_path, self.print_segments

    def update_plot(self):
        # Update robot and airbrush positions
        self.robot_circle.center = self.simulator.robot_pos
        self.airbrush_circle.center = self.simulator.airbrush_pos
        self.axis_rectangle.set_xy(self.simulator.robot_pos - [AXIS_X_LIMIT, AXIS_Y_LIMIT])

        # Update robot path
        robot_path = np.array(self.simulator.robot_points)
        self.robot_path.set_data(robot_path[:, 0], robot_path[:, 1])

        # Update print segments
        segments = self.simulator.print_segments + [self.simulator.current_print_segment]
        self.print_segments.set_segments([segment for segment in segments if len(segment) > 1])

    def update_robot_data(self):
        for key, label in self.robot_data_labels.items():
            value = getattr(self.robot_speed.robot_data, key, 0.0)
            label.config(text=f"{value:.3f}")
        
        # Update other labels
        robot_pos = self.simulator.robot_pos if self.simulator else (0, 0)
        self.robot_pos_label.config(text=f"({robot_pos[0]:.2f}, {robot_pos[1]:.2f})")
        
        airbrush_pos = self.simulator.airbrush_pos if self.simulator else (0, 0)
        self.airbrush_pos_label.config(text=f"({airbrush_pos[0]:.2f}, {airbrush_pos[1]:.2f})")
        
        progress = (self.simulator.current_target_index / len(self.simulator.target_points)) * 100 if self.simulator else 0
        self.progress_label.config(text=f"{progress:.1f}%")
        
        speed = np.linalg.norm([self.robot_speed.command_req.vx, self.robot_speed.command_req.vy])
        self.speed_label.config(text=f"{speed:.2f} m/s")
        
        self.after(100, self.update_robot_data)  # Update every 100ms

def main():
    rclpy.init()
    robot_speed = RobotSpeed()
    
    # Create and start ROS2 spin thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(robot_speed,))
    ros_thread.start()
    
    # Create and run Tkinter application
    app = RobotControlUI(robot_speed)
    app.mainloop()
    
    # Cleanup
    rclpy.shutdown()
    ros_thread.join()

if __name__ == "__main__":
    main()
    

#Parameters
DEFAULT_FILENAME = os.path.abspath('src/corosols/corosols/gcode/Lines2.txt')

# Parmeters to vary
#point_factor = 1000
#max_speed = 0.1 # Maximum speed in m/s
#min_speed = 0.02  # Minimum speed in m/s

# Fixed Parameters
robot_size = 0.5  # Robot size in meters (50 cm)
axis_size = (0.24, 0.16)  # 2D axis size in meters (24 cm x 16 cm)
axis_precision = 0.01  # 2D axis precision in meters (2 mm)
AXIS_X_LIMIT = 0.114  # 12 cm in meters
AXIS_Y_LIMIT = 0.0325  # 6 cm in meters

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    
def parse_gcode(filename,point_factor):
    points = []
    printing = []
    current_pos = [0, 0]
    is_printing = False
    current_mode = 'G00'

    with open(filename, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith('G00'):
                current_mode = 'G00'
                is_printing = False
            elif line.startswith('G01'):
                current_mode = 'G01'
                is_printing = True
            elif line.startswith('G02') or line.startswith('G03'):
                current_mode = line[:3]
                is_printing = True
            elif line == 'ON':
                is_printing = True
            elif line == 'OFF':
                is_printing = False
            
            if not (line in ['%', '()', 'M30'] or line.startswith('G90') or line.startswith('G17') or line.startswith('G21')):
                if line in ['ON','OFF']:
                    pass
                else:
                    parts = line.split()
                    new_pos = current_pos.copy()
                    center_offset = [0, 0]
                    for part in parts:
                        if part.startswith('X'):
                            new_pos[0] = float(part[1:])
                        elif part.startswith('Y'):
                            new_pos[1] = float(part[1:])
                        elif part.startswith('I'):
                            center_offset[0] = float(part[1:])
                        elif part.startswith('J'):
                            center_offset[1] = float(part[1:])
                    
                    if current_mode in ['G02', 'G03']:
                        center = [current_pos[0] + center_offset[0], current_pos[1] + center_offset[1]]
                        circle_points = generate_circle_points(current_pos, new_pos, center, current_mode,point_factor)
                        for point in circle_points:
                            points.append((point, 'G01', None))
                            printing.append(is_printing)
                    elif new_pos != current_pos or (current_mode in ['G02', 'G03'] and (center_offset[0] != 0 or center_offset[1] != 0)):
                        points.append((new_pos.copy(), current_mode, center_offset))
                        printing.append(is_printing)
                        current_pos = new_pos
                    
                    current_pos = new_pos

    return points, printing

def generate_circle_points(start, end, center, mode,point_factor):
    radius = np.linalg.norm(np.array(center) - np.array(start))
    num_points = int (radius * point_factor)
    start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])
    
    if mode == 'G02':  # Clockwise
        if end_angle >= start_angle:
            end_angle -= 2 * np.pi
            
    elif mode == 'G03':  # G03, Counter-clockwise
        if start_angle >= end_angle:
            end_angle += 2 * np.pi
    
    angles = np.linspace(start_angle, end_angle, num_points)
    np.sort(angles)
    
    circle_points = []
    
    for i in range (1,len(angles)):
            x = center[0] + radius * np.cos(angles[i])
            y = center[1] + radius * np.sin(angles[i])
            circle_points.append([x, y]) 
    return circle_points

class UnreachableAxisMovementError(Exception):
    pass

def is_axis_movement_reachable(current_pos, target_pos):
    movement = np.array(target_pos) - np.array(current_pos)
    return abs(movement[0]) <= AXIS_X_LIMIT and abs(movement[1]) <= AXIS_Y_LIMIT


def calculate_robot_speed_vector(current_pos, prev_target_pos, target_pos, max_speed, min_speed, dt, pid_controller):
    # Calculate desired path vector
    path_vector = np.array(target_pos) - np.array(prev_target_pos)
    path_length = np.linalg.norm(path_vector)
    
    if path_length == 0:
        # Prevent division by zero if the path length is too small (targets are the same)
        return np.array([0, 0])
    
    normalized_path_vector = path_vector / path_length

    # Calculate current position vector relative to previous target
    current_vector = np.array(current_pos) - np.array(prev_target_pos)

    # Project current position onto the path
    projection = np.dot(current_vector, normalized_path_vector) * normalized_path_vector

    # Calculate error (perpendicular distance from path)
    error_vector = current_vector - projection
    error = np.linalg.norm(error_vector)
    
    # Determine sign of error (which side of the path the robot is on)
    error_sign = np.sign(np.cross(normalized_path_vector, error_vector))

    # Use PID controller to calculate correction
    correction = pid_controller.update(error * error_sign, dt)

    # Calculate direction to target
    direction_vector = np.array(target_pos) - np.array(current_pos)
    distance = np.linalg.norm(direction_vector)
    
    if distance == 0:
        # Avoid division by zero if the robot is at the target
        return np.array([0, 0])
    
    normalized_direction_vector = direction_vector / distance

    # Calculate speed based on distance to target
    raw_speed = distance / dt

    # Apply speed limits, preserving direction
    speed = np.clip(raw_speed, -max_speed, max_speed)
    if abs(speed) < min_speed:
        speed = min_speed if speed >= 0 else -min_speed

    # Combine path following and error correction
    perpendicular_vector = np.array([-normalized_path_vector[1], normalized_path_vector[0]])
    corrected_direction = normalized_direction_vector - correction * perpendicular_vector

    # Normalize corrected direction, but ensure it's not a zero vector
    corrected_direction_norm = np.linalg.norm(corrected_direction)
    if corrected_direction_norm > 0:
        corrected_direction /= corrected_direction_norm

    # Calculate final speed vector
    speed_vector = corrected_direction * speed

    return speed_vector


def move_with_speed_vector(start_pos, speed_vector, dt, error_std=0, ratio=[1,1], publisher=None):
    # Adjust robot position based on speed vector and ratio, applying time step
    new_pos = np.array(start_pos) + speed_vector * ratio * dt
    return new_pos


def calculate_projection_point(prev_target, current_target, current_pos, robot_pos, is_printing):
    v = np.array(current_target) - np.array(prev_target)
    u = np.array(robot_pos) - np.array(prev_target)

    v_norm_sq = np.dot(v, v)
    if v_norm_sq == 0:
        # Avoid division by zero if prev_target and current_target are too close
        return np.array([0, 0])
    
    # Projection of current robot position onto the path
    t = np.dot(u, v) / v_norm_sq
    projection_point = np.array(prev_target) + t * v - robot_pos
    
    move_to_target = current_target - (robot_pos + projection_point)

    # Calculate scaling factor based on axis limits
    factor = np.max(np.abs(np.divide(move_to_target, np.array([AXIS_X_LIMIT, AXIS_Y_LIMIT]))))

    # Apply scaling based on proximity to the target
    if np.linalg.norm(current_target - robot_pos) < AXIS_Y_LIMIT:
        if factor > 1:
            move_to_target = move_to_target / factor  
    else:
        move_to_target = np.array([0, 0])

    if is_printing:
        move_to_target = projection_point + move_to_target

    # Recalculate scaling factor after adjustments
    factor = np.max(np.abs(np.divide(move_to_target, np.array([AXIS_X_LIMIT, AXIS_Y_LIMIT]))))
    if factor > 1:
        move_to_target = move_to_target / factor  

    return move_to_target

