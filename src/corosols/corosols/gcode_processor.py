import numpy as np

#Parameters
point_factor = 1000
filename = "/home/hamzalegion/COROSOLS_WS/src/corosols/corosols/gcode/Lines2.txt"
dt = 0.01  # time step for simulation in seconds (50ms)
max_speed = 0.1  # Maximum speed in m/s
min_speed = 0.05  # Minimum speed in m/s
max_axis_speed = 2  # Maximum speed in m/s for the 2D axis
movement_error_std = 0.03  # standard deviation of the robot movement error (3 cm/s)
robot_size = 0.5  # Robot size in meters (50 cm)
axis_size = (0.24, 0.16)  # 2D axis size in meters (24 cm x 16 cm)
axis_precision = 0.002  # 2D axis precision in meters (2 mm)
AXIS_X_LIMIT = 0.114  # 12 cm in meters
AXIS_Y_LIMIT = 0.06  # 8 cm in meters


class GCodeProcessor:
    def __init__(self):
        self.point_factor = 1000
        self.dt = 0.01  # time step for simulation in seconds (50ms)
        self.max_speed = 0.5  # Maximum speed in m/s
        self.min_speed = 0.05  # Minimum speed in m/s
        self.max_axis_speed = 2  # Maximum speed in m/s for the 2D axis
        self.movement_error_std = 0.03  # standard deviation of the robot movement error (3 cm/s)
        self.robot_size = 0.5  # Robot size in meters (50 cm)
        self.axis_size = (0.24, 0.16)  # 2D axis size in meters (24 cm x 16 cm)
        self.axis_precision = 0.002  # 2D axis precision in meters (2 mm)
        self.AXIS_X_LIMIT = 0.114  # 12 cm in meters
        self.AXIS_Y_LIMIT = 0.06  # 8 cm in meters

    class UnreachableAxisMovementError(Exception):
        pass

    def parse_gcode(self, filename):
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
                            circle_points = self.generate_circle_points(current_pos, new_pos, center, current_mode)
                            for point in circle_points:
                                points.append((point, 'G01', None))
                                printing.append(is_printing)
                        elif new_pos != current_pos or (current_mode in ['G02', 'G03'] and (center_offset[0] != 0 or center_offset[1] != 0)):
                            points.append((new_pos.copy(), current_mode, center_offset))
                            printing.append(is_printing)
                            current_pos = new_pos
                        
                        current_pos = new_pos

        return points, printing

    def generate_circle_points(self, start, end, center, mode):
        radius = np.linalg.norm(np.array(center) - np.array(start))
        num_points = int(radius * self.point_factor)
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
        
        for i in range(1, len(angles)):
                x = center[0] + radius * np.cos(angles[i])
                y = center[1] + radius * np.sin(angles[i])
                circle_points.append([x, y]) 
        return circle_points

    def is_axis_movement_reachable(self, current_pos, target_pos):
        movement = np.array(target_pos) - np.array(current_pos)
        return abs(movement[0]) <= self.AXIS_X_LIMIT and abs(movement[1]) <= self.AXIS_Y_LIMIT

    def calculate_robot_speed_vector(self, current_pos, target_pos, max_speed, min_speed):
        direction_vector = np.array(target_pos) - np.array(current_pos)
        distance = np.linalg.norm(direction_vector)
        if distance == 0:
            return np.array([0, 0])
        normalized_direction_vector = direction_vector / distance
        speed = max(min(max_speed, distance / self.dt), min_speed)
        speed_vector = normalized_direction_vector * speed
        return speed_vector

    def move_with_speed_vector(self, start_pos, speed_vector, dt, error_std=0, ratio=[1,1], publisher=None):
        new_pos = np.array(start_pos) + (speed_vector)*ratio * dt 
        return new_pos

    def calculate_projection_point(self, prev_target, current_target, current_pos):
        move_to_target = np.clip(current_target - current_pos, [-0.02, -0.02], [0.02, 0.02])/2
        v = np.array(current_target) - np.array(prev_target)                                                                                                         
        u = np.array(current_pos) - np.array(prev_target)
        t = np.dot(u, v) / np.dot(v, v)
        projection_point = np.array(prev_target) + t * v
        return projection_point + move_to_target

    def calculate_axis_speed_vector(self, current_pos, target_pos, max_speed):
        movement = np.array(target_pos) - np.array(current_pos)
        
        movement[0] = np.clip(current_pos[0] + movement[0], -self.AXIS_X_LIMIT, self.AXIS_X_LIMIT) - current_pos[0]
        movement[1] = np.clip(current_pos[1] + movement[1], -self.AXIS_Y_LIMIT, self.AXIS_Y_LIMIT) - current_pos[1]
        distance = np.linalg.norm(movement)
        if distance == 0:
            return np.array([0, 0])
        normalized_direction_vector = movement / distance
        speed = min(max_speed, distance / self.dt)
        speed_vector = normalized_direction_vector * speed
        return speed_vector