#!/usr/bin/env python3
"""
Test script to verify forward kinematics at home position
"""
import math

# Constants from serial_read.py
MOTOR_X_OFFSET = 315.0
MOTOR_Y_POS = 0.0
M1_POS = (-MOTOR_X_OFFSET, MOTOR_Y_POS)  # (-315, 0)
M2_POS = (MOTOR_X_OFFSET, MOTOR_Y_POS)   # (315, 0)
AIRBRUSH_OFFSET = 35.0
AIRBRUSH_TO_ROBOT_CENTER_OFFSET = 145.0  # mm
ROD1_init_L = 377
ROD2_init_L = 377

def forward_kinematics(l1, l2):
    x1, y1 = M1_POS  # (-315, 0)
    x2, y2 = M2_POS  # (315, 0)
    
    d = math.hypot(x2 - x1, y2 - y1)  # Distance between motors = 630mm
    
    if d > l1 + l2 or d < abs(l1 - l2) or d == 0:
        return None
    
    # Circle intersection to find pivot point P
    a = (l1**2 - l2**2 + d**2) / (2 * d)
    h = math.sqrt(max(0, l1**2 - a**2))
    
    x_mid = x1 + a * (x2 - x1) / d
    y_mid = y1 + a * (y2 - y1) / d
    
    # Pivot point P (choose positive Y solution)
    P_x = x_mid + h * (y2 - y1) / d
    P_y = y_mid + h * (x2 - x1) / d  # Changed minus to plus for positive Y
    
    # Step 2: Calculate angle of rod 2 (from M2 to P)
    angle_rod2 = math.atan2(P_y - y2, P_x - x2)
    
    # Step 3: Pen is 35mm perpendicular to rod 2 in +Y direction
    airbrush_x = P_x + AIRBRUSH_OFFSET * math.cos(angle_rod2 + math.pi/2)
    airbrush_y = P_y + AIRBRUSH_OFFSET * math.sin(angle_rod2 + math.pi/2)
    
    return airbrush_x, airbrush_y

# Test at home position
print("=== Testing Forward Kinematics at Home Position ===")
print(f"M1_POS = {M1_POS}")
print(f"M2_POS = {M2_POS}")
print(f"ROD1_init_L = {ROD1_init_L}")
print(f"ROD2_init_L = {ROD2_init_L}")
print(f"AIRBRUSH_TO_ROBOT_CENTER_OFFSET = {AIRBRUSH_TO_ROBOT_CENTER_OFFSET}")
print()

# Home position
l1 = ROD1_init_L
l2 = ROD2_init_L
result = forward_kinematics(l1, l2)

if result:
    airbrush_x, airbrush_y = result
    print(f"At home (l1={l1}, l2={l2}):")
    print(f"  Raw FK result: x={airbrush_x:.4f}mm, y={airbrush_y:.4f}mm")
    
    # What stepper_x and stepper_y would be
    stepper_x = -airbrush_x / 1000.0
    stepper_y = (airbrush_y - AIRBRUSH_TO_ROBOT_CENTER_OFFSET) / 1000.0
    print(f"  stepper_x = {stepper_x:.4f}m = {stepper_x*1000:.2f}mm")
    print(f"  stepper_y = {stepper_y:.4f}m = {stepper_y*1000:.2f}mm")
    print()
    
    # What AIRBRUSH_TO_ROBOT_CENTER_OFFSET should be for stepper_y = 0
    correct_offset = airbrush_y
    print(f"  To get stepper_y = 0 at home, AIRBRUSH_TO_ROBOT_CENTER_OFFSET should be: {correct_offset:.2f}mm")
else:
    print("FK returned None - impossible geometry")

# Test at target position (stepper_x = 0.04, stepper_y = 0)
print("\n=== Testing at Target Position (stepper_x=40mm, stepper_y=0mm) ===")
# This requires inverse kinematics, which we'll skip for now
