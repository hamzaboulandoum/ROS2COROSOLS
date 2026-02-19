import os
import sys
from tkinter import filedialog, simpledialog
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle, Circle
from matplotlib.collections import LineCollection
from matplotlib.patches import FancyArrow
import time
import tkinter as tk
from tkinter import ttk
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from custom_interfaces.msg import SerialData
from custom_interfaces.srv import Commands
import serial
import csv 
import matplotlib
import json

RUN_WITH_STATION = False
TEST_2D_PLAN = True
SETTINGS_FILE = "settings.json"

cmap = matplotlib.cm.get_cmap('Spectral')
class Logger:
    def __init__(self, log_file):
        self.log_file = log_file
        # Initialize CSV writer
        self.file = open(self.log_file, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['id','current_pos', 'corrected_direction', 'prev_target_pos', 'target_pos', 'speed_vector'])
        self.id = 0
    def log_step(self, *args):
        # Log relevant data for debugging and analysis
        self.writer.writerow( [self.id] + [arg for arg in args])
        self.id += 1
    def close(self):
        self.file.close()

class PIDLogger:
    def __init__(self, log_file):
        self.log_file = log_file
        # Initialize CSV writer
        self.file = open(self.log_file, mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time','dt', 'thetaerr', 'thetaout', 'vrcmd', 'kp', 'ki', 'kd'])
    def log_pid_data(self, dt, theta_error, theta_output, vr_command, kp, ki, kd):
        # Log relevant data for debugging and analysis
        timestamp = time.time()
        self.writer.writerow([timestamp, dt, theta_error, theta_output, vr_command, kp, ki, kd])
    def close(self):
        self.file.close()

class Simulator:
    def __init__(self, target_points, printing, robot_speed, point_factor=1000, max_speed=0.1, min_speed=0.02):
        self.start_pos = robot_speed.getOdoData()
        self.target_points = target_points
        self.printing = printing
        self.robot_pos = self.start_pos.copy()
        self.airbrush_pos = self.start_pos.copy() 
        self.current_target_index = 0
        self.robot_points = [self.start_pos.copy()]
        self.print_segments = []
        self.current_print_segment = []
        self.robot_velocities = []
        self.axis_velocities = []
        self.relative_airbrush_pos = np.array([0.0,0.0])
        self.robot_speed = robot_speed
        self.robot_pid_controllerX = PIDController(kp=15, ki=0.5, kd=0.0)
        self.robot_pid_controller_theta = PIDController(kp=0.04, ki=0.0005, kd=0.005)
        self.prev_time = time.time()
        self.current_error_colors = []
        self.error_colors = []
        # New parameters
        self.point_factor = point_factor
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.airbrush_acivation = True 
        self.logger = Logger('simulator_log.csv')
        self.pid_logger = PIDLogger('theta_pid_log.csv')
        self.error = 0
        self.max_error = 0
        self.average_error = 0
        self.N_error = 0
        self.simulation_status = False
        self.servo_min = 50
        self.servo_max = 100
        self.compressor_min = 50
        self.compressor_max = 100
        self.counter = 0
        # Speed control mode: 0 = direct target seeking (simple), 1 = PID path following
        self.speed_control_mode = 0
        # Axis stop threshold: stop robot when target is within this fraction of axis limits
        self.axis_stop_threshold = 0.7
        self.prev_printing_state = False
    def simulate(self): 
        while self.simulation_status and self.step():
            time.sleep(0.005)  # 5ms delay to prevent CPU overload and allow other threads
    def step(self, reset_target=False):
        if self.current_target_index >= len(self.target_points):
            self.robot_speed.command_req.vy = 0.0
            self.robot_speed.command_req.vx = 0.0
            self.robot_speed.command_req.vr = 0.0
            self.robot_speed.command_req.airbrush = 0
            self.robot_speed.command_req.ab_servo = 0
            self.robot_speed.command_req.stepperx = 0.0
            self.robot_speed.command_req.steppery = 0.0
            self.robot_speed.send_speed()
            return False

        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time
        theta_error = self.robot_speed.theta
        theta_error = 0.0
        current_target, _, _ = self.target_points[self.current_target_index]
        self.station_speeds = (self.robot_speed.getOdoData()-self.robot_pos)/dt
        self.robot_pos = self.robot_speed.getOdoData() -np.array([0,0.300])+ rotate_vector(0,0.300,(theta_error)*np.pi/180)
        current_target = np.array(current_target)
        is_printing = self.printing[self.current_target_index]        ############################
        if self.current_target_index < len(self.target_points) - 1:
            self.next_target = np.array(self.target_points[self.current_target_index + 1][0])
        else:
            self.next_target = self.start_pos
        
        ############################
        if self.current_target_index > 0:
            prev_target = np.array(self.target_points[self.current_target_index - 1][0])
        else:
            # For the first target, use (0,0) as the starting point to match G-code origin
            prev_target = np.array([0.0, 0.0])
        ############################
        if self.current_target_index > 1:
            before_previous_target = np.array(self.target_points[self.current_target_index - 2][0])
        else:
            before_previous_target = self.start_pos

        if reset_target:
            current_target = np.array([0,0])
            is_printing = False
            prev_target = self.robot_pos
        
        # Linear movement
        #30cm et 1,4cm        
        self.airbrush_pos = self.robot_pos + rotate_vector(self.robot_speed.robot_data.stepper_x,self.robot_speed.robot_data.stepper_y,(theta_error)*np.pi/180)
        #self.robot_speed.get_logger().info(f"sx = {self.robot_speed.robot_data.stepper_x}  <> sy = {self.robot_speed.robot_data.stepper_y}")
        #self.robot_speed.get_logger().info(f"sx2 = ...")
        
        
        # Debug logging
        #self.robot_speed.get_logger().info(f"prev={prev_target}, curr={current_target}, airbrush={self.airbrush_pos}, stepper_target={self.relative_airbrush_pos}")
        
        # Check if target is within axis reach (based on axis_stop_threshold)
        target_relative_to_robot = current_target - self.robot_pos
        target_within_axis_x = abs(target_relative_to_robot[0]) < AXIS_X_LIMIT * self.axis_stop_threshold
        target_within_axis_y = abs(target_relative_to_robot[1]) < AXIS_Y_LIMIT * self.axis_stop_threshold
        target_within_axis_range = target_within_axis_x and target_within_axis_y
        
        # Calculate robot velocity based on mode and axis range check
        if target_within_axis_range:
            # Target is within axis reach - stop robot, let steppers do the work
            robot_velocity = np.array([0.0, 0.0])
            #self.robot_speed.get_logger().info(f"Target within axis range - robot stopped, steppers working")
        elif self.speed_control_mode == 0:
            # Mode 0: Direct target seeking at max speed (simple, original logic)
            robot_velocity = calculate_robot_speed_vector_direct(
                np.array(self.robot_speed.getOdoData()),
                current_target,
                self.max_speed,
                self.min_speed,
                logger=self.robot_speed.get_logger()
            )
        else:
            # Mode 1: PID-based path following (existing complex logic)
            robot_velocity = calculate_robot_speed_vector(
                    self.robot_pos , 
                    prev_target,
                    current_target, 
                    self.max_speed,
                    self.min_speed,
                    dt,
                    self.robot_pid_controllerX,
                    logger = self.robot_speed.get_logger(),
                    mode = 1,
                    next_target = self.next_target,
                    before_previous=before_previous_target,
                    is_printing = is_printing
                )
        self.relative_airbrush_pos,self.error = calculate_projection_point(prev_target,
                                                                current_target, 
                                                                self.airbrush_pos, 
                                                                self.robot_pos, 
                                                                is_printing, 
                                                                logger=self.robot_speed.get_logger(),
                                                                vx=robot_velocity[0],
                                                                vy=robot_velocity[1],
                                                                max_speed=self.max_speed,
                                                                dt=dt)
        robot_velocity=rotate_vector(robot_velocity[0],robot_velocity[1],-(theta_error)*np.pi/180)
        self.robot_velocities.append(robot_velocity)
        if is_printing:
            self.max_error = max(self.max_error,self.error)
            self.average_error = (self.average_error*self.N_error + self.error)/(self.N_error+1)
            self.N_error += 1
        
        theta_output = self.robot_pid_controller_theta.update(theta_error, dt)
        vr_command = np.clip(theta_output, -0.2, 0.2)
        self.robot_speed.command_req.vr = vr_command
        
        '''self.pid_logger.log_pid_data(
            dt=dt,
            theta_error=theta_error,
            theta_output=theta_output,
            vr_command=vr_command,
            kp=self.robot_pid_controller_theta.kp,
            ki=self.robot_pid_controller_theta.ki,
            kd=self.robot_pid_controller_theta.kd
        )'''
        airbrush_output = self.compressor_min + (self.compressor_max - self.compressor_min) * np.linalg.norm(robot_velocity)/0.3
        self.robot_speed.command_req.vy = -robot_velocity[0]#*ratio[0]/error)
        self.robot_speed.command_req.vx = robot_velocity[1]#*ratio[1]/error)
        self.robot_speed.command_req.airbrush = int(is_printing*airbrush_output)
        self.robot_speed.command_req.ab_servo = is_printing*self.servo_max
        self.logger.log_step(self.robot_pos, robot_velocity,[self.robot_speed.command_req.vx,self.robot_speed.command_req.vy], [self.robot_speed.odoData.twist.twist.linear.x,self.robot_speed.odoData.twist.twist.linear.y], prev_target,self.error)        
        self.robot_points.append(self.robot_pos.copy())
        
        if is_printing:
            self.current_print_segment.append(self.airbrush_pos.copy())
            self.current_error_colors.append(self.error)
        elif self.current_print_segment:
            self.print_segments.append(self.current_print_segment)
            self.error_colors.append(self.current_error_colors)
            self.current_print_segment = []
            self.current_error_colors = []
        
        # Check if we've reached the target.
        if np.linalg.norm(self.airbrush_pos - current_target) <= axis_precision:
            self.check_multi_targets(self.current_target_index)
            self.current_target_index += 1
        
        if reset_target and np.linalg.norm(self.robot_pos - current_target) <= 0.025:
            self.robot_speed.command_req.vy = 0.0
            self.robot_speed.command_req.vx = 0.0
            self.robot_speed.command_req.vr = 0.0
            self.robot_speed.command_req.airbrush = 0
            self.robot_speed.command_req.ab_servo = 0
            self.robot_speed.send_speed()
            return False
        
        self.robot_speed.command_req.stepperx = float(self.relative_airbrush_pos[0])
        self.robot_speed.command_req.steppery = float(self.relative_airbrush_pos[1])
        if not self.airbrush_acivation:
            self.robot_speed.command_req.airbrush = 0
            self.robot_speed.command_req.ab_servo = 0
        
        # Send speed without blocking
        self.robot_speed.send_speed()
        
        # Debug log with current target index
        #self.robot_speed.get_logger().info(f"dt={dt}")
        if is_printing != self.prev_printing_state:
            time.sleep(0.1)
        self.prev_printing_state = is_printing
        return True
    def check_multi_targets(self,target_index):
        N=0
        return 0
def minn(a,b):
    l=[]
    for i in range(0,len(a)):
        if a[i]<=b[i]:
            l.append(a[i])
        else :
            l.append(b[i])
    return l
class RobotSpeed(Node):
    def __init__(self):
        super().__init__('robot_speed')
        self.x_offset = 0
        self.y_offset = 0
        self.x_reverse = 1
        self.y_reverse = 1
        # Station status - initially disconnected
        self.station_status = False 
        self.prism_status = False 
        
        self.odom_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_listener, 1)
        self.robot_data_subscription = self.create_subscription(
            SerialData, 'robot_data', self.robot_data_listener, 1)
        self.parameters_publisher = self.create_publisher(String, 'robot_params', 10)
        self.parameters_subscription = self.create_subscription(String, 'robot_params', self.params_listener, 10)

        # Use publisher instead of service client for non-blocking communication
        self.command_publisher = self.create_publisher(Commands.Request, 'commands_topic', 10)
        
        # Keep the command request object for compatibility
        self.command_req = Commands.Request()
        
        # Getting Data from Odom
        self.odoData = Odometry()
        self.robot_data = SerialData()
        self.u = np.zeros(2)
        self.theta = 0
          # Heading fusion variables
        self.imu_heading = None  # Raw IMU heading (degrees) - None until first reading
        self.imu_heading_offset = 0.0  # Offset to correct IMU drift
        self.corrected_heading = 0.0  # Final corrected heading
        
        # Station-based heading calculation
        self.last_station_pos = None
        self.last_station_time = None
        self.commanded_velocity_sum = np.array([0.0, 0.0])  # Sum of commanded velocities (vx, vy)
        self.commanded_velocity_count = 0  # Number of velocity samples
        self.last_robot_data_time = time.time()
        
        # Minimum displacement for heading correction (50mm)
        self.min_displacement_for_heading = 0.05
        
        # Heading correction filter (low-pass)
        self.heading_correction_alpha = 0.1  # How fast to apply corrections
    def params_listener(self, msg):
        data = msg.data
        params = data.split(';')
        if params[0] == 'Station_status':
            if params[1] == '1':
                self.station_status = True
                self.prism_status = True
            elif params[1] == '0':
                self.station_status = False
        elif params[0] == 'Station_error':
            if params[1] == 'E1':
                self.prism_status = False
            elif params[1] == 'E2':
                tk.messagebox.showwarning(title="Station error: Status E2", message=params[2])
                self.prism_status = False
            elif params[1] == 'E3':
                tk.messagebox.showwarning(title="Station error: Status E3", message=params[2])
                self.prism_status = False

    def odom_listener(self, msg):
        try:
            self.odoData = msg
            # Don't automatically set station_status here - let it be controlled by params_listener
            
            # Station-based heading correction
            current_pos = np.array([
                self.x_reverse * (msg.pose.pose.position.x - self.x_offset),
                self.y_reverse * (msg.pose.pose.position.y - self.y_offset)
            ])
            current_time = time.time()
            
            if self.last_station_pos is not None and self.last_station_time is not None:
                # Calculate actual displacement from station
                station_displacement = current_pos - self.last_station_pos
                station_displacement_magnitude = np.linalg.norm(station_displacement)
                
                # Only use for heading correction if displacement is significant AND we have velocity data
                if station_displacement_magnitude > self.min_displacement_for_heading and self.commanded_velocity_count > 0:
                    # Calculate actual movement direction from station (degrees)
                    station_direction = np.degrees(np.arctan2(station_displacement[1], station_displacement[0]))
                    
                    # Calculate average commanded velocity direction
                    avg_commanded_velocity = self.commanded_velocity_sum / self.commanded_velocity_count
                    commanded_magnitude = np.linalg.norm(avg_commanded_velocity)
                    
                    if commanded_magnitude > 0.01:  # Only if we commanded significant movement
                        # Direction of commanded velocity (what we told the robot to do)
                        commanded_direction = np.degrees(np.arctan2(avg_commanded_velocity[1], avg_commanded_velocity[0]))
                        
                        # Heading error = actual direction - commanded direction
                        # If robot is facing correctly, these should match
                        heading_error = station_direction - commanded_direction
                        
                        # Normalize to [-180, 180]
                        while heading_error > 180:
                            heading_error -= 360
                        while heading_error < -180:
                            heading_error += 360
                        
                        # Apply correction to IMU offset (low-pass filtered)
                        self.imu_heading_offset += self.heading_correction_alpha * heading_error
                        
                        # Clamp offset to reasonable range [-180, 180]
                        while self.imu_heading_offset > 180:
                            self.imu_heading_offset -= 360
                        while self.imu_heading_offset < -180:
                            self.imu_heading_offset += 360
                        
                        # Log correction for debugging
                        self.get_logger().debug(
                            f"Heading correction: error={heading_error:.2f}°, "
                            f"offset={self.imu_heading_offset:.2f}°, "
                            f"station_disp={station_displacement_magnitude*1000:.1f}mm"
                        )
            
            # Reset for next cycle
            self.last_station_pos = current_pos.copy()
            self.last_station_time = current_time
            self.commanded_velocity_sum = np.array([0.0, 0.0])
            self.commanded_velocity_count = 0
            
        except Exception as e:
            self.get_logger().error(f"Error in odom_listener: {e}")
    def robot_data_listener(self, msg):
        current_time = time.time()
        dt = current_time - self.last_robot_data_time
        self.last_robot_data_time = current_time
        
        self.robot_data = msg
        
        # Initialize heading offset on first IMU reading
        if self.imu_heading is None:
            self.imu_heading_offset = -self.robot_data.heading
            
        # Get IMU heading (in degrees)
        self.imu_heading = self.robot_data.heading
        
        # Apply corrected heading (IMU + drift correction) - result in degrees
        self.theta = self.imu_heading + self.imu_heading_offset
        
        # Normalize theta to [-180, 180]
        while self.theta > 180:
            self.theta -= 360
        while self.theta < -180:
            self.theta += 360
        
        # Accumulate commanded velocities for heading correction
        # These are the velocities we commanded to the robot (in world frame)
        # We'll use command_req.vx and command_req.vy which are what we sent to STM
        # Note: vx/vy mapping: command_req.vy = -robot_velocity[0], command_req.vx = robot_velocity[1]
        # So robot_velocity[0] = -command_req.vy, robot_velocity[1] = command_req.vx
        commanded_vx = -self.command_req.vy  # Undo the mapping
        commanded_vy = self.command_req.vx   # Undo the mapping
        
        self.commanded_velocity_sum[0] += commanded_vx
        self.commanded_velocity_sum[1] += commanded_vy
        self.commanded_velocity_count += 1
        
    def send_speed(self):
        """Publish command using topic (non-blocking)"""
        try:
            self.command_publisher.publish(self.command_req)
        except Exception as e:
            self.get_logger().error(f'Error sending speed: {e}')
    
    def getOdoData(self):
        return np.array([self.x_reverse*(self.odoData.pose.pose.position.x-self.x_offset), self.y_reverse*(self.odoData.pose.pose.position.y-self.y_offset)])
    
    def get_heading(self):
        return self.odoData.Odom.pose.pose.orientation.z
        
def rclspin(node):
    try:
        rclpy.spin(node)
    except Exception:
        node.destroy_node()
        

class RobotControlUI(tk.Tk):
    def load_settings(self):
        if os.path.exists(SETTINGS_FILE):
            with open(SETTINGS_FILE, "r") as f:
                try:
                    return json.load(f)
                except Exception:
                    return {}
        return {}

    def save_settings(self):
        settings = {
            "station_ip": self.station_ip.get(),
            "station_port": self.station_port.get(),
            "point_factor": self.point_factor.get(),
            "max_speed": self.max_speed.get(),
            "min_speed": self.min_speed.get(),
            "servo_min": getattr(self.simulator, "servo_min", 50),
            "servo_max": getattr(self.simulator, "servo_max", 100),
            "compressor_min": getattr(self.simulator, "compressor_min", 50),
            "compressor_max": getattr(self.simulator, "compressor_max", 100),
            "filename_var": self.filename_var.get(),
        }
        with open(SETTINGS_FILE, "w") as f:
            json.dump(settings, f, indent=4)

    def __init__(self, robot_speed):
        super().__init__()
        self.title("Robot Control UI")
        self.robot_speed = robot_speed
        self.simulator = None
        self.animation = None
        self.is_paused = True
        self.is_plotting = False
        self.file_selected = True
        self.init_pos = False
        self.station_status = tk.BooleanVar(value=False)
        self.station_status.trace_add('write', self.update_station_button)
        self.station_ip = tk.StringVar()
        self.station_port = tk.StringVar()
        self.point_factor = tk.DoubleVar()
        self.max_speed = tk.DoubleVar()
        self.min_speed = tk.DoubleVar()
        self.theta_kp = tk.DoubleVar()
        self.theta_ki = tk.DoubleVar()
        self.theta_kd = tk.DoubleVar()
        self.filename_var = tk.StringVar()

        # Load settings before creating UI elements
        settings = self.load_settings()
        self.station_ip.set(settings.get("station_ip", "192.168.30.101"))
        self.station_port.set(settings.get("station_port", "1212"))
        self.point_factor.set(settings.get("point_factor", 1000))
        self.max_speed.set(settings.get("max_speed", 0.1))
        self.min_speed.set(settings.get("min_speed", 0.02))
        self.filename_var.set(settings.get("filename_var", DEFAULT_FILENAME))

        self.arrows = []
        self.point_factor.trace_add('write', self.on_point_factor_change)
        self.max_speed.trace_add('write', self.on_speed_change)
        self.min_speed.trace_add('write', self.on_speed_change)
        self.configure(bg='#1e1e1e')
        self.geometry('1400x800')
        self.attributes('-fullscreen', True)
        self.bind('<Escape>', self.toggle_fullscreen)
        self.create_ui_elements()
        self.init_simulation()
        self.update_robot_data()
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.airbrush_start_stop = False
    
    def toggle_fullscreen(self, event=None):
        self.attributes('-fullscreen', not self.attributes('-fullscreen'))
        
    def on_closing(self):
        os._exit(0)
        
    def on_point_factor_change(self, *args):
        self.save_settings()
        self.init_simulation()

    def on_speed_change(self, *args):
        self.save_settings()
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

        # Create menu bar
        menu_bar = tk.Menu(self, background='#1e1e1e', foreground='#ffffff', activebackground='#005fb3', activeforeground='#ffffff')
        self.config(menu=menu_bar)

        

        # Configuration menu
        self.config_menu = tk.Menu(menu_bar, tearoff=0, background='#1e1e1e', foreground='#ffffff')
        menu_bar.add_cascade(label="Configuration", menu=self.config_menu)

        # Add Manual Control button to menu bar
        menu_bar.add_command(label="Manual Control", command=self.open_manual_control)

        # Station Settings submenu
        self.station_menu = tk.Menu(self.config_menu, tearoff=0, background='#1e1e1e', foreground='#ffffff')
        self.config_menu.add_cascade(label="Station Settings", menu=self.station_menu )
        self.station_menu.add_command(label="Set Station IP", command=self.set_station_ip)
        self.station_menu.add_command(label="Set Port", command=self.set_station_port)
        self.station_menu.add_command(label="Connect", command=self.connect_to_station)

        # Other configuration commands
        self.config_menu.add_separator()
        self.config_menu.add_command(label="Reset Origin", command=self.reset_origin)
        self.config_menu.add_command(label="Default Origin", command=self.default_origin)
        self.config_menu.add_command(label="Reverse X", command=self.reverse_x)
        self.config_menu.add_command(label="Reverse Y", command=self.reverse_y)
        self.config_menu.add_command(label="Default Orientation", command=self.default_orientation)
        self.config_menu.add_separator()
        self.config_menu.add_command(label="Reset Angles", command=self.reset_angles)
        self.config_menu.add_command(label="Default Angles", command=self.default_angles)
        self.config_menu.add_separator()
        self.config_menu.add_command(label="Airbrush limits", command=self.airbrush_limits)

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
    def airbrush_limits(self):
        dialog = self.AirbrushLimitsDialog(self, title="Airbrush Limits",servo_min_default=self.simulator.servo_min,servo_max_default=self.simulator.servo_max,compressor_min_default=self.simulator.compressor_min,compressor_max_default=self.simulator.compressor_max)
        if dialog.result:
            self.simulator.servo_min= dialog.result["servo_min"]
            self.simulator.servo_max = dialog.result["servo_max"]
            self.simulator.compressor_min = dialog.result["compressor_min"]
            self.simulator.compressor_max = dialog.result["compressor_max"]
            self.save_settings()
    class AirbrushLimitsDialog(simpledialog.Dialog):
        def __init__(self, parent, title=None, servo_min_default=0, servo_max_default=0, compressor_min_default=0, compressor_max_default=0):
            self.servo_min_default = servo_min_default
            self.servo_max_default = servo_max_default
            self.compressor_min_default = compressor_min_default
            self.compressor_max_default = compressor_max_default
            super().__init__(parent, title)
        def body(self, master):
            tk.Label(master, text="Servo Min:").grid(row=0)
            tk.Label(master, text="Servo Max:").grid(row=1)
            tk.Label(master, text="Compressor Min:").grid(row=2)
            tk.Label(master, text="Compressor Max:").grid(row=3)

            self.servo_min = tk.Entry(master)
            self.servo_max = tk.Entry(master)
            self.compressor_min = tk.Entry(master)
            self.compressor_max = tk.Entry(master)

            self.servo_min.insert(0, str(self.servo_min_default))
            self.servo_max.insert(0, str(self.servo_max_default))
            self.compressor_min.insert(0, str(self.compressor_min_default))
            self.compressor_max.insert(0, str(self.compressor_max_default))


            self.servo_min.grid(row=0, column=1)
            self.servo_max.grid(row=1, column=1)
            self.compressor_min.grid(row=2, column=1)
            self.compressor_max.grid(row=3, column=1)

            return self.servo_min  # initial focus

        def apply(self):
            self.result = {
                "servo_min": int(self.servo_min.get()),
                "servo_max": int(self.servo_max.get()),
                "compressor_min": int(self.compressor_min.get()),
                "compressor_max": int(self.compressor_max.get())
            }
    def set_station_ip(self):
        ip = simpledialog.askstring("Station IP", "Enter Station IP:", initialvalue=self.station_ip.get())
        if ip:
            self.station_ip.set(ip)
            self.save_settings()

    def open_manual_control(self):
        """Open the manual control window"""
        control_window = ManualControlWindow(self, self.robot_speed)
        control_window.transient(self)  # Make window modal
        control_window.grab_set()  # Make window modal
    def set_station_port(self):
        port = simpledialog.askstring("Station Port", "Enter Station Port:", initialvalue=self.station_port.get())
        if port:
            self.station_port.set(port)
            self.save_settings()
    def reset_angles(self):
        msg = String()
        msg.data = "angles_offset;1"
        self.robot_speed.parameters_publisher.publish(msg)
    def default_angles(self):
        msg = String()
        msg.data = "angles_offset;0"
        self.robot_speed.parameters_publisher.publish(msg)
    def reset_origin(self):
        self.robot_speed.x_offset = self.robot_speed.odoData.pose.pose.position.x
        self.robot_speed.y_offset = self.robot_speed.odoData.pose.pose.position.y
    def default_origin(self):
        self.robot_speed.x_offset = 0
        self.robot_speed.y_offset = 0
    def reverse_x(self):
        self.robot_speed.x_reverse = -self.robot_speed.x_reverse
        if self.robot_speed.x_reverse == -1:
            self.config_menu.entryconfig(4,label = "X reversed")
        else:
            self.config_menu.entryconfig(4,label = "Reverse X")

    def reverse_y(self):
        self.robot_speed.y_reverse = -self.robot_speed.y_reverse
        if self.robot_speed.y_reverse == -1:
            self.config_menu.entryconfig(5,label = "Y reversed")
        else:
            self.config_menu.entryconfig(5,label = "Reverse Y")
    def default_orientation(self):
        self.robot_speed.x_reverse = 1
        self.robot_speed.y_reverse = 1
        self.reverse_x_button.config(text="Reverse X")
        self.reverse_y_button.config(text="Reverse Y")
    def connect_to_station(self):
        msg = String()
        msg.data = f"ip;{self.station_ip.get()}"
        self.robot_speed.parameters_publisher.publish(msg)
        msg.data = f"port;{self.station_port.get()}"
        self.robot_speed.parameters_publisher.publish(msg)
        msg.data = "Connect_to_station"
        self.robot_speed.parameters_publisher.publish(msg)
    
    def update_station_button(self, *args):
        if self.station_status.get():
            self.station_menu.entryconfig(2,label = "Connected")
        else:
            self.station_menu.entryconfig(2,label = "Connect")
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
            self.robot_speed.command_req.airbrush = 100
            self.robot_speed.command_req.ab_servo = 100
        else:
            self.airbrush_button.config(text="Start Airbrush")
            self.robot_speed.command_req.airbrush = 0
            self.robot_speed.command_req.ab_servo = 0
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
        
        value_label = tk.Label(frame, text=value_text, bg=bg_color, fg=value_fg_color, width=25, anchor='e')
        value_label.pack(side=tk.LEFT, padx=5)
        
        return label, value_label

    def create_info_labels(self, parent):
        info_frame = ttk.Frame(parent)
        info_frame.pack(fill=tk.X, pady=10)
        
        _, self.robot_pos_label = self.create_styled_label(info_frame, "Robot Position:")
        _, self.airbrush_pos_label = self.create_styled_label(info_frame, "Airbrush Position:")
        _, self.progress_label = self.create_styled_label(info_frame, "Progress:")
        _, self.speed_label = self.create_styled_label(info_frame, "Robot Speed:")

        _, self.error = self.create_styled_label(info_frame, "Actual error:")
        _, self.max_error = self.create_styled_label(info_frame, "Maximum error:")
        _, self.avg_error = self.create_styled_label(info_frame, "Average error:")
        
        # Labels for additional robot data
        robot_data_frame = ttk.Frame(info_frame)
        robot_data_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(robot_data_frame, text="Robot Data:", font=('Helvetica', 12, 'bold')).pack(anchor='w', padx=5, pady=5)
        
        self.robot_data_labels = {}
        for data in ['power_voltage', 'stepper_x', 'stepper_y']:
            _, self.robot_data_labels[data] = self.create_styled_label(robot_data_frame, f"{data.replace('_', ' ').title()}:")
        
        imu_data_frame = ttk.Frame(info_frame)
        imu_data_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(imu_data_frame, text="IMU Data:", font=('Helvetica', 12, 'bold')).pack(anchor='w', padx=5, pady=5)

        self.imu_data_labels = {}
        for data in ['roll', 'pitch', 'heading', 'X speed', 'Y speed', 'Z speed']:
            _, self.imu_data_labels[data] = self.create_styled_label(imu_data_frame, f"{data.replace('_', ' ').title()}:")

    def browse_file(self):
        filename = filedialog.askopenfilename(filetypes=[("G-code files", "*.txt"), ("All files", "*.*")],initialdir=os.path.abspath('src/corosols/corosols/gcode'))
        if filename:
            self.filename_var.set(filename)
            self.file_selected = True
            self.save_settings()
            self.init_simulation()

    def init_simulation(self):
        filename = self.filename_var.get()
        target_points, printing = parse_gcode(filename,self.point_factor.get())
        self.simulator = Simulator(target_points, printing, self.robot_speed,
                                   point_factor=self.point_factor.get(),
                                   max_speed=self.max_speed.get(),
                                   min_speed=self.min_speed.get())
        self.init_plot()
        if hasattr(self, "_pending_airbrush_settings"):
            s = self._pending_airbrush_settings
            if "servo_min" in s:
                self.simulator.servo_min = s["servo_min"]
            if "servo_max" in s:
                self.simulator.servo_max = s["servo_max"]
            if "compressor_min" in s:
                self.simulator.compressor_min = s["compressor_min"]
            if "compressor_max" in s:
                self.simulator.compressor_max = s["compressor_max"]
            del self._pending_airbrush_settings

    def toggle_simulation(self):
        if not self.file_selected:
            return

        if self.is_paused:
            self.start_simulation()
        else:
            self.pause_simulation()

    def start_simulation(self):
        self.is_paused = False
        self.init_pos = False
        self.start_button.config(text="Pause")
        self.stop_button.config(state=tk.NORMAL)
        self.simulator.simulation_status = True
        # Start simulation in daemon thread so it doesn't block shutdown
        threading.Thread(target=self.simulator.simulate, daemon=True).start()
        if self.animation is None:
            self.animation = FuncAnimation(self.fig, self.animate, interval=100, blit=True)

    def pause_simulation(self):
        self.is_paused = True
        self.start_button.config(text="Resume")
        self.robot_speed.command_req.vx = 0.0
        self.robot_speed.command_req.vy = 0.0
        self.robot_speed.command_req.vr = 0.0
        self.robot_speed.command_req.airbrush = 0
        self.robot_speed.command_req.ab_servo = 0
        self.simulator.simulation_status = False
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
            self.robot_speed.command_req.vr = 0.0
            self.robot_speed.command_req.airbrush = 0
            self.robot_speed.command_req.ab_servo = 0
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
        self.robot_circle = Circle(self.simulator.robot_pos, 0.005, fc='#ff0000', ec='#ff0000', label='Robot')
        self.airbrush_circle = Circle(self.simulator.airbrush_pos, 0.01, fc='#00a8e8', ec='#00a8e8', label='Airbrush')
        self.axis_rectangle = Rectangle(self.simulator.robot_pos - [AXIS_X_LIMIT, AXIS_Y_LIMIT], 
                                        2*AXIS_X_LIMIT, 2*AXIS_Y_LIMIT, fill=False, edgecolor='#ff8c00', linestyle='--')
        self.robot_path, = self.ax.plot([], [], color='#ff0000', linewidth=0.5, alpha=0.5, label='Robot Path')
        self.print_segments = LineCollection([], cmap="Reds", linewidths=2, label='Print Path')

        self.ax.add_patch(self.robot_circle)
        self.ax.add_patch(self.airbrush_circle)
        self.ax.add_patch(self.axis_rectangle)
        self.ax.add_collection(self.print_segments)
        self.canvas.draw()
    def plot_update_thread(self):
        while True:
            self.update_plot()
            time.sleep(0.1)
    def animate(self, frame):
        if self.simulator is None or self.is_paused:
            return self.robot_circle, self.airbrush_circle, self.axis_rectangle, self.robot_path, self.print_segments

        if True :
            self.update_plot()
        else:
            self.stop_simulation()

        return self.robot_circle, self.airbrush_circle, self.axis_rectangle, self.robot_path, self.print_segments
    def draw_speed_arrows(self):
        # Clear previous arrows
        for arrow in getattr(self, 'arrows', []):
            arrow.remove()
        self.arrows = []

        # Get the robot's current speed
        vx = self.robot_speed.command_req.vx
        vy = self.robot_speed.command_req.vy

        # Define the starting point of the arrows (center of the robot square)
        x, y = self.simulator.robot_pos

        # Create arrows
        arrow_vx = FancyArrow(x, y, -vy, 0, color='red', width=0.001)
        arrow_vy = FancyArrow(x, y, 0, vx, color='blue', width=0.001)
        arrow_v = FancyArrow(x, y, -vy, vx, color='green', width=0.001)
        # Add arrows to the plot
        self.ax.add_patch(arrow_vx)
        self.ax.add_patch(arrow_vy)
        self.ax.add_patch(arrow_v)
        
        # Store arrows to remove them later
        self.arrows.append(arrow_vx)
        self.arrows.append(arrow_vy)
        self.arrows.append(arrow_v)
    def update_plot(self):
        # Update robot and airbrush positions
        
        self.robot_circle.center = self.simulator.robot_pos
        self.airbrush_circle.center = self.simulator.airbrush_pos
        self.axis_rectangle.set_xy(self.simulator.robot_pos - [AXIS_X_LIMIT, AXIS_Y_LIMIT])

        # Update robot path
        robot_path = np.array(self.simulator.robot_points)
        self.robot_path.set_data(robot_path[:, 0], robot_path[:, 1])
        

        self.draw_speed_arrows()
        # Create point-to-point segments and corresponding colors
        all_segments = []
        all_colors = []

        # Process existing segments
        for main_segment, main_error_color in zip(self.simulator.print_segments, self.simulator.error_colors):
            if len(main_segment) > 1:
                # Process each print_segment as if it were a current_print_segment
                for i in range(len(main_segment) - 1):
                    # Create segment pair
                    segment_pair = np.array([main_segment[i], main_segment[i + 1]]).reshape(1, 2, 2)
                    all_segments.append(segment_pair[0])
                    
                    # Handle error color
                    if isinstance(main_error_color, (list, np.ndarray)):
                        # Use corresponding error color if it's an array
                        color = main_error_color[i]
                    else:
                        # Use the single color value
                        color = main_error_color
                    all_colors.append(color)

        # Process current segment if it exists
        if len(self.simulator.current_print_segment) > 1:
            for i in range(len(self.simulator.current_print_segment) - 1):
                # Create segment pair
                segment_pair = np.array([
                    self.simulator.current_print_segment[i],
                    self.simulator.current_print_segment[i + 1]
                ]).reshape(1, 2, 2)
                all_segments.append(segment_pair[0])
                
                # Handle error color
                if isinstance(self.simulator.current_error_colors, (list, np.ndarray)):
                    color = self.simulator.current_error_colors[i]
                else:
                    color = self.simulator.current_error_colors
                all_colors.append(color)

        # Update the LineCollection if we have segments
        if all_segments:
            # Convert to numpy arrays
            segments_array = np.array(all_segments)
            colors_array = np.array(all_colors)

            # Create color mapping
            cmap = plt.get_cmap("Reds")
            if len(colors_array) > 0:
                # Normalize color values
                norm = plt.Normalize(vmin=min(colors_array), vmax=max(colors_array))
                colors = cmap(norm(colors_array))

                # Update the LineCollection
                self.print_segments.set_segments(segments_array)
                self.print_segments.set_color(colors)
        self.canvas.draw()

    def update_robot_data(self):
        for key, label in self.robot_data_labels.items():
            value = getattr(self.robot_speed.robot_data, key, 0.0)
            label.config(text=f"{value:.3f}")

        X_speed = self.robot_speed.odoData.twist.twist.linear.x
        Y_speed = self.robot_speed.odoData.twist.twist.linear.y
        Z_speed = self.robot_speed.odoData.twist.twist.linear.z

        self.imu_data_labels['X speed'].config(text=f"{X_speed:.3f}")
        self.imu_data_labels['Y speed'].config(text=f"{Y_speed:.3f}")
        self.imu_data_labels['Z speed'].config(text=f"{Z_speed:.3f}")

        roll = self.robot_speed.odoData.pose.pose.orientation.x
        pitch = self.robot_speed.odoData.pose.pose.orientation.y
        heading = self.robot_speed.theta
        self.imu_data_labels['roll'].config(text=f"{roll:.3f}")
        self.imu_data_labels['pitch'].config(text=f"{pitch:.3f}")
        self.imu_data_labels['heading'].config(text=f"{heading:.3f}")

        # Update other labels
        if self.station_status.get() != self.robot_speed.station_status:
            self.station_status.set(self.robot_speed.station_status)  

        if (not self.robot_speed.prism_status or not self.robot_speed.station_status )and self.is_paused == False and RUN_WITH_STATION:
            self.pause_simulation()
        
        robot_pos = self.robot_speed.getOdoData()
        self.robot_pos_label.config(text=f"({robot_pos[0]:.3f}, {robot_pos[1]:.3f})")
        
        airbrush_pos = self.simulator.airbrush_pos if self.simulator else (0, 0)
        self.airbrush_pos_label.config(text=f"({airbrush_pos[0]:.3f}, {airbrush_pos[1]:.3f})")
        
        progress = (self.simulator.current_target_index / len(self.simulator.target_points)) * 100 if self.simulator else 0
        self.progress_label.config(text=f"{progress:.1f}%")
        
        speed = np.linalg.norm([self.robot_speed.command_req.vx, self.robot_speed.command_req.vy])
        self.speed_label.config(text=f"{speed:.3f} m/s")

        self.error.config(text=f"{self.simulator.error:.1f} mm")
        self.max_error.config(text=f"{self.simulator.max_error:.1f} mm")
        self.avg_error.config(text=f"{self.simulator.average_error:.1f} mm")
        
        self.after(100, self.update_robot_data)  # Update every 100ms
class ManualControlWindow(tk.Toplevel):
    def __init__(self, parent, robot_speed):
        super().__init__(parent)
        self.title("Manual Control")
        self.robot_speed = robot_speed
        
        # Set window properties
        self.geometry("400x500")
        self.configure(background='#1e1e1e')
        
        # Create main frame
        main_frame = ttk.Frame(self)
        main_frame.pack(expand=True, fill='both', padx=20, pady=20)
        
        # Create speed selection frame
        speed_frame = ttk.Frame(main_frame)
        speed_frame.pack(fill='x', pady=(0, 20))
        
        # Speed selector
        ttk.Label(speed_frame, text="Speed:").pack(side='left', padx=(0, 10))
        self.speed_var = tk.StringVar(value="1.0")
        speed_combo = ttk.Combobox(speed_frame, textvariable=self.speed_var, values=["1.0", "0.5","0.2"], 
                                  state='readonly', width=10)
        speed_combo.pack(side='left')
        
        # Create button frames
        controls_frame = ttk.Frame(main_frame)
        controls_frame.pack(expand=True)
        
        # Create and pack the arrow buttons in a grid layout
        # Forward button (top)
        self.create_arrow_button(controls_frame, "?", 0, 1, 
            lambda: self.move_robot(vx=float(self.speed_var.get())))
        
        # Left button (middle left)
        self.create_arrow_button(controls_frame, "<", 1, 0,
            lambda: self.move_robot(vy=float(self.speed_var.get())))
        
        # Stop button (middle)
        self.create_stop_button(controls_frame, 1, 1)
        
        # Right button (middle right)
        self.create_arrow_button(controls_frame, ">", 1, 2,
            lambda: self.move_robot(vy=-float(self.speed_var.get())))
        
        # Backward button (bottom)
        self.create_arrow_button(controls_frame, "?", 2, 1,
            lambda: self.move_robot(vx=-float(self.speed_var.get())))
        
        # Rotation controls
        rotation_frame = ttk.Frame(main_frame)
        rotation_frame.pack(fill='x', pady=20)
        
        # Rotate CCW button (left)
        self.create_arrow_button(rotation_frame, "?", 0, 0,
            lambda: self.move_robot(vr=float(self.speed_var.get())), pack=True)
            
        # Rotate CW button (right)
        self.create_arrow_button(rotation_frame, "?", 0, 1,
            lambda: self.move_robot(vr=-float(self.speed_var.get())), pack=True)

    def create_arrow_button(self, parent, text, row=None, column=None, command=None, pack=False):
        button = ttk.Button(parent, text=text, width=3)
        button.bind('<Button-1>', lambda e: command())  # Press
        button.bind('<ButtonRelease-1>', lambda e: self.stop_robot())  # Release
        
        if pack:
            button.pack(side='left', padx=10, expand=True)
        else:
            button.grid(row=row, column=column, padx=5, pady=5)
        
        return button
    
    def create_stop_button(self, parent, row, column):
        button = ttk.Button(parent, text="�", width=3)
        button.bind('<Button-1>', lambda e: self.stop_robot())
        button.grid(row=row, column=column, padx=5, pady=5)
        return button
    
    def move_robot(self, vx=0.0, vy=0.0, vr=0.0):
        """Send movement command to robot"""
        self.robot_speed.get_logger().info(f"Moving robot: vx={vx}, vy={vy}, vr={vr}")
        self.robot_speed.command_req.stepperx = 0.0
        self.robot_speed.command_req.steppery = 0.0
        self.robot_speed.command_req.vx = vx
        self.robot_speed.command_req.vy = vy
        self.robot_speed.command_req.vr = vr
        self.robot_speed.send_speed()
    
    def stop_robot(self):
        """Stop all robot movement"""
        self.robot_speed.command_req.vx = 0.0
        self.robot_speed.command_req.vy = 0.0
        self.robot_speed.command_req.vr = 0.0
        self.robot_speed.get_logger().info(f"Moving robot: vx={0}, vy={0}, vr={0}")
        self.robot_speed.send_speed()
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
axis_precision = 0.001  # 2D axis precision in meters (2 mm)
AXIS_X_LIMIT = 0.150
AXIS_Y_LIMIT = 0.06

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

def calculate_robot_speed_vector_direct(current_pos, target_pos, max_speed, min_speed, logger=None):
    """
    Simple direct target-seeking speed calculation.
    Robot moves directly towards the target at max speed with ramp-down near target.
    
    Args:
        current_pos (array-like): Current robot position [x, y]
        target_pos (array-like): Target position [x, y]
        max_speed (float): Maximum allowed speed
        min_speed (float): Minimum allowed speed
        logger: Optional logger object
    
    Returns:
        numpy.ndarray: Speed vector [vx, vy]
    """
    current_pos = np.array(current_pos, dtype=float)
    target_pos = np.array(target_pos, dtype=float)
    
    # Minimum distance threshold (1mm)
    MIN_DISTANCE = 0.001
    
    # Calculate vector to target
    to_target = target_pos - current_pos
    distance = np.linalg.norm(to_target)
    
    # If we've reached the target, return zero velocity
    if distance < MIN_DISTANCE:
        return np.zeros(2)
    
    # Normalize direction
    direction = to_target / distance
    
    # Calculate speed based on distance (slow down when approaching target)
    # Use a simple linear ramp-down when within 200mm of target
    ramp_distance = 0.2  # 200mm
    if distance < ramp_distance:
        speed = min_speed + (max_speed - min_speed) * (distance / ramp_distance)
    else:
        speed = max_speed
    
    # Clamp speed between min and max
    speed = np.clip(speed, min_speed, max_speed)
    
    return speed * direction

def calculate_robot_speed_vector(current_pos, prev_target_pos, target_pos, max_speed, min_speed, dt, pid_controller,
                               logger=None, mode=1,next_target = np.zeros(2),before_previous = np.zeros(2),is_printing = False):
    """
    Calculate robot speed vector with perpendicular error correction for path following.
    Added checks to ensure correct path direction and prevent backwards movement.
    
    Args:
        current_pos (array-like): Current robot position [x, y]
        prev_target_pos (array-like): Previous target position [x, y]
        target_pos (array-like): Current target position [x, y]
        max_speed (float): Maximum allowed speed
        min_speed (float): Minimum allowed speed
        dt (float): Time step
        pid_controller: PID controller object for perpendicular error correction
        logger: Optional logger object
        mode (int): Control mode (1: normal, 2: precise)
    
    Returns:
        numpy.ndarray: Speed vector [vx, vy]
    """
    # Convert inputs to numpy arrays
    current_pos = np.array(current_pos, dtype=float)
    prev_target_pos = np.array(prev_target_pos, dtype=float)
    target_pos = np.array(target_pos, dtype=float)
      # Minimum distance threshold (1mm) - below this we consider positions equal
    MIN_DISTANCE = 0.001  # 1mm in meters
    
    # Calculate path vector and normalize it
    path_vector = (target_pos - prev_target_pos)
    path_length = np.linalg.norm(path_vector)
    
    # If path segment is too short, no movement needed
    if path_length < MIN_DISTANCE:
        return np.zeros(2)
    
    path_direction = path_vector / path_length

    # Calculate vector from current position to target
    to_target = target_pos - current_pos
    to_target_length = np.linalg.norm(to_target)

    # If we've reached the target (within 1mm), return zero velocity
    if to_target_length < MIN_DISTANCE:
        return np.zeros(2)

    from_target = current_pos - prev_target_pos
    from_target_length = np.linalg.norm(from_target)

    # Check if we're headed in the wrong direction
    # by comparing the dot product of path direction and direction to target
    #direction_alignment = np.dot(path_direction, to_target / to_target_length) if to_target_length > 1e-6 else 1.0
    
    # If we're headed in significantly wrong direction, adjust path direction
    relative_pos = current_pos - prev_target_pos
    
    # Calculate perpendicular vector to path (right-hand normal)
    perpendicular_direction = np.array([-path_direction[1], path_direction[0]])
    
    # Calculate parallel and perpendicular components
    parallel_dist = np.dot(relative_pos, path_direction)
    perp_dist = np.dot(relative_pos, perpendicular_direction)
    
    # Calculate signed perpendicular error
    signed_error = perp_dist
    
    # Get PID correction for perpendicular error
    correction = pid_controller.update(signed_error, dt)
    correction = np.clip(correction,-1,1)
    
    # Calculate base forward speed based on progress and error
    # Modified error damping to be less aggressive
    if is_printing:
        error_damping = 1.0
        # error_damping = np.exp(-abs(signed_error)*50 * 0.5)  # Made damping less aggressive # Made damping less aggressive
    else:
        error_damping = 1.0
    
    # Add distance-based speed scaling
    
    next_path_vector = next_target - target_pos
    before_previous_vector = before_previous - prev_target_pos

    angle = angle_between_vectors_np(path_vector, next_path_vector)
    angle_factor = angle / np.pi

    angle2 = angle_between_vectors_np(before_previous_vector, path_vector)
    angle_factor2 = angle2 / np.pi

    distance_factor = 1.0
    if angle_factor < 0.2:
        if from_target_length < max_speed*0.8 and np.linalg.norm(before_previous_vector) > 0 and np.linalg.norm(path_vector) > 0:
            distance_factor = 1-angle_factor2**0.7*(1-from_target_length / max_speed/0.8)**0.2
            
        if to_target_length < max_speed*0.8 and np.linalg.norm(next_path_vector) > 0 and np.linalg.norm(path_vector) > 0:
            distance_factor = 1-angle_factor**0.7*(1-to_target_length / max_speed/0.8)**0.2
    else:
        if from_target_length < max_speed*0.8 and np.linalg.norm(before_previous_vector) > 0 and np.linalg.norm(path_vector) > 0:
            distance_factor = 1-angle_factor2**0.1*(1-from_target_length / max_speed/0.8)**0.2
            
        if to_target_length < max_speed*0.8 and np.linalg.norm(next_path_vector) > 0 and np.linalg.norm(path_vector) > 0:
            distance_factor = 1-angle_factor**0.1*(1-to_target_length / max_speed/0.8)**0.2
    #logger.info(f"Distance factor: {distance_factor:.3f} error: {signed_error:.3f} error_damping: {error_damping:.3f} , from_target_length: {from_target_length:.3f}")
    
    base_speed = min(max_speed,
                    max(min_speed,
                        max_speed * error_damping * distance_factor))
    if from_target_length < 0.05 or to_target_length< 0.05:
        #base_speed = 0.04
        #correction=0
        pass
    # Forward component: along the path
    forward_component = base_speed * to_target/to_target_length
    
    # Corrective component: perpendicular to path
    correction_speed = -correction * base_speed # Reduced correction intensity
    correction_component = correction_speed * perpendicular_direction
    
    # Combine components for final speed vector
    speed_vector = forward_component + correction_component
    '''
    # Add direction verification
    if np.dot(speed_vector, to_target) < 0 and to_target_length > min_speed * dt:
        # Project speed vector onto d
        # irection to target
        speed_vector = np.dot(speed_vector, to_target/to_target_length) * to_target/to_target_length'''
    #speed_vector = np.dot(speed_vector, to_target/to_target_length) * to_target/to_target_length
    # Limit the final speed vector magnitude
    speed_magnitude = np.linalg.norm(speed_vector)
    if speed_magnitude > max_speed:
        speed_vector = (speed_vector / speed_magnitude) * max_speed
    elif speed_magnitude < min_speed and speed_magnitude > 0:
        speed_vector = (speed_vector / speed_magnitude) * min_speed
    #logger.info(f"vx{speed_vector[0]}    vy{speed_vector[1]}")
    #return np.array([0.0,0.0])
    return speed_vector

def angle_between_vectors_np(u, v,signed = False):
    u = np.array(u)
    v = np.array(v)
    
    # Check for zero/tiny vectors to avoid division by zero (1mm threshold)
    norm_u = np.linalg.norm(u)
    norm_v = np.linalg.norm(v)
    if norm_u < 0.001 or norm_v < 0.001:
        return 0.0
    
    # Compute the angle using the dot product
    cos_theta = np.dot(u, v) / (norm_u * norm_v)
    angle_rad = np.arccos(np.clip(cos_theta, -1.0, 1.0))
    
    # Determine the sign using the cross product in 2D
    if signed:
        cross_product = u[0] * v[1] - u[1] * v[0]
        if cross_product < 0:
            angle_rad = -angle_rad
    
    return angle_rad
def rotate_vector(vx, vy, theta):
    """
    Rotates a single 2D vector (vx, vy) by an angle theta (in radians).

    Parameters:
    vx: float
        x-component of the vector.
    vy: float
        y-component of the vector.
    theta: float
        Rotation angle in radians, expected to be between -pi and pi.

    Returns:
    vx_rotated, vy_rotated: float, float
        Rotated x and y components.
    """
    # Rotation matrix components
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    # Apply the rotation matrix
    vx_rotated = cos_theta * vx - sin_theta * vy
    vy_rotated = sin_theta * vx + cos_theta * vy
    
    return np.array([vx_rotated, vy_rotated])

def calculate_projection_point(prev_target, current_target, airbrush_pos, robot_pos, is_printing, logger=None, vx=0, vy=0, max_speed=0.1, dt=1):
    """
    Calcule la position cible des steppers pour que l'airbrush suive la trajectoire.
    
    Strategy:
    - When robot is MOVING: Keep steppers centered (perpendicular correction only)
      This gives margin for when robot stops
    - When robot is STOPPED and target within axis: Move toward target in small steps
    """
    prev_target = np.array(prev_target)
    current_target = np.array(current_target)
    airbrush_pos = np.array(airbrush_pos)
    robot_pos = np.array(robot_pos)
    current_speed = np.linalg.norm(np.array([vx, vy]))
    
    # 1. Vecteur de la trajectoire
    path_vector = current_target - prev_target
    path_length_sq = np.dot(path_vector, path_vector)
    if path_length_sq < 1e-10:
        return airbrush_pos - robot_pos, 0

    path_length = np.sqrt(path_length_sq)
    path_direction = path_vector / path_length
    perpendicular_direction = np.array([-path_direction[1], path_direction[0]])

    # 2. Projection de l'airbrush sur la trajectoire
    airbrush_to_path_start = airbrush_pos - prev_target
    t_airbrush = np.dot(airbrush_to_path_start, path_vector) / path_length_sq
    t_airbrush_clamped = np.clip(t_airbrush, 0, 1)
    projection_on_path = prev_target + t_airbrush_clamped * path_vector
    
    # 3. Erreur perpendiculaire (distance airbrush <-> ligne de trajectoire)
    perp_error = np.dot(airbrush_to_path_start, perpendicular_direction)
    
    # 4. Check conditions
    target_within_axis = (abs(current_target[0] - robot_pos[0]) < AXIS_X_LIMIT and 
                          abs(current_target[1] - robot_pos[1]) < AXIS_Y_LIMIT)
      # Robot is considered "stopped" when speed is very low (less than 10mm/s)
    robot_is_stopped = current_speed < 0.01

    # Error correction factor - only correct 90% of error at a time to prevent overshooting
    ERROR_CORRECTION_FACTOR = 0.9
    
    # 5. Calculate stepper target based on robot state
    
    if robot_is_stopped and target_within_axis:
        # Robot stopped and target within reach - move toward target in small steps
        to_target = current_target - airbrush_pos
        distance_to_target = np.linalg.norm(to_target)

        # Maximum step size when stopped (2mm per cycle for smoother motion)
        MAX_STEP_SIZE = 0.001  # 2mm

        if distance_to_target > MAX_STEP_SIZE:
            step_direction = to_target / distance_to_target
            # Apply error correction factor - only move 80% of the way
            incremental_target = airbrush_pos + step_direction * MAX_STEP_SIZE * ERROR_CORRECTION_FACTOR
            stepper_target = incremental_target - robot_pos
        else:
            # Close to target - apply 80% correction
            stepper_target = (current_target - robot_pos) * ERROR_CORRECTION_FACTOR + (airbrush_pos - robot_pos) * (1 - ERROR_CORRECTION_FACTOR)
    else:
        # Robot is moving - track the projection point on path with 80% correction
        current_stepper_pos = airbrush_pos - robot_pos
        target_stepper_pos = projection_on_path - robot_pos
        # Blend current position with target position (80% toward target)
        stepper_target = current_stepper_pos + (target_stepper_pos - current_stepper_pos) * ERROR_CORRECTION_FACTOR
    
    # 6. Limitation physique des axes
    stepper_target[0] = np.clip(stepper_target[0], -AXIS_X_LIMIT, AXIS_X_LIMIT)
    stepper_target[1] = np.clip(stepper_target[1], -AXIS_Y_LIMIT, AXIS_Y_LIMIT)
    
    return stepper_target, abs(perp_error) * 1000