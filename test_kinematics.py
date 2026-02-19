#!/usr/bin/env python3
import math

# Constants from serial_read.py
MOTOR_X_OFFSET = 315.0
MOTOR_Y_POS = 0.0
ROD1_init_L = 377
ROD2_init_L = 377
M1_POS = (-MOTOR_X_OFFSET, MOTOR_Y_POS)
M2_POS = (MOTOR_X_OFFSET, MOTOR_Y_POS)
AIRBRUSH_OFFSET = 35.0
AIRBRUSH_TO_ROBOT_CENTER_OFFSET = 145.0

def inverse_kinematics(x_airbrush, y_airbrush):
    dx = x_airbrush - M2_POS[0]
    dy = y_airbrush - M2_POS[1]
    d_airbrush = math.hypot(dx, dy)
    if d_airbrush < AIRBRUSH_OFFSET:
        return ROD1_init_L, ROD2_init_L
    d_rod2 = math.sqrt(d_airbrush**2 - AIRBRUSH_OFFSET**2)
    angle_to_pen = math.atan2(dy, dx)
    angle_offset = math.atan2(AIRBRUSH_OFFSET, d_rod2)
    angle_rod2 = angle_to_pen - angle_offset
    P_x = M2_POS[0] + d_rod2 * math.cos(angle_rod2)
    P_y = M2_POS[1] + d_rod2 * math.sin(angle_rod2)
    l1 = math.hypot(P_x - M1_POS[0], P_y - M1_POS[1])
    l2 = d_rod2
    return l1, l2

def forward_kinematics(l1, l2):
    x1, y1 = M1_POS
    x2, y2 = M2_POS
    d = math.hypot(x2 - x1, y2 - y1)
    if d > l1 + l2 or d < abs(l1 - l2) or d == 0:
        return None
    a = (l1**2 - l2**2 + d**2) / (2 * d)
    h = math.sqrt(max(0, l1**2 - a**2))
    x_mid = x1 + a * (x2 - x1) / d
    y_mid = y1 + a * (y2 - y1) / d
    P_x = x_mid + h * (y2 - y1) / d
    P_y = y_mid + h * (x2 - x1) / d
    angle_rod2 = math.atan2(P_y - y2, P_x - x2)
    airbrush_x = P_x + 35 * math.cos(angle_rod2 + math.pi/2)
    airbrush_y = P_y + 35 * math.sin(angle_rod2 + math.pi/2)
    return airbrush_x, airbrush_y

# Test case from your logs
print("=" * 70)
print("TEST 1: Round-trip with exact values (no quantization)")
print("=" * 70)

x_target_m = 0.04
y_target_m = 0.000
x_mm = x_target_m * 1000
y_mm = y_target_m * 1000 + AIRBRUSH_TO_ROBOT_CENTER_OFFSET

print(f"Input position: x={x_mm}mm, y={y_mm}mm")

# IK
l1, l2 = inverse_kinematics(x_mm, y_mm)
print(f"IK result: l1={l1:.4f}mm, l2={l2:.4f}mm")

# FK with exact values
result = forward_kinematics(l1, l2)
if result:
    ax, ay = result
    print(f"FK result: x={ax:.4f}mm, y={ay:.4f}mm")
    print(f"Round-trip error: x={ax - x_mm:.4f}mm, y={ay - y_mm:.4f}mm")

print("\n" + "=" * 70)
print("TEST 2: With quantization (* 100 / 100)")
print("=" * 70)

# Simulate quantization
l1_delta = l1 - ROD1_init_L
l2_delta = l2 - ROD2_init_L
print(f"Delta sent: l1_delta={l1_delta:.4f}mm, l2_delta={l2_delta:.4f}mm")

# Quantize to 0.01mm (int16 * 100)
l1_delta_q = round(l1_delta * 100) / 100
l2_delta_q = round(l2_delta * 100) / 100
print(f"Quantized delta: l1_delta={l1_delta_q:.2f}mm, l2_delta={l2_delta_q:.2f}mm")

l1_q = l1_delta_q + ROD1_init_L
l2_q = l2_delta_q + ROD2_init_L
print(f"Quantized lengths: l1={l1_q:.2f}mm, l2={l2_q:.2f}mm")

result_q = forward_kinematics(l1_q, l2_q)
if result_q:
    ax_q, ay_q = result_q
    sx_q = ax_q / 1000.0
    sy_q = (ay_q - AIRBRUSH_TO_ROBOT_CENTER_OFFSET) / 1000.0
    print(f"FK quantized result: x={ax_q:.4f}mm, y={ay_q:.4f}mm")
    print(f"Position error: x={ax_q - x_mm:.4f}mm, y={ay_q - y_mm:.4f}mm")
    print(f"Output in meters: x={sx_q:.6f}m, y={sy_q:.6f}m")

print("\n" + "=" * 70)
print("TEST 3: Using your actual log values")
print("=" * 70)

# From your logs:
# Sent: l1=400.5922, l2=304.8059
# Received: l1=398.39, l2=304.82

l1_sent = 400.5922
l2_sent = 304.8059
l1_recv = 398.39
l2_recv = 304.82

print(f"Sent: l1={l1_sent}, l2={l2_sent}")
print(f"Received: l1={l1_recv}, l2={l2_recv}")
print(f"Difference: l1={l1_recv - l1_sent:.4f}mm, l2={l2_recv - l2_sent:.4f}mm")

# What position does the sent value give?
result_sent = forward_kinematics(l1_sent, l2_sent)
if result_sent:
    print(f"\nFK from sent: x={result_sent[0]:.4f}mm, y={result_sent[1]:.4f}mm")
    print(f"  -> meters: x={result_sent[0]/1000:.6f}m, y={(result_sent[1]-AIRBRUSH_TO_ROBOT_CENTER_OFFSET)/1000:.6f}m")

# What position does the received value give?
result_recv = forward_kinematics(l1_recv, l2_recv)
if result_recv:
    print(f"FK from recv: x={result_recv[0]:.4f}mm, y={result_recv[1]:.4f}mm")
    print(f"  -> meters: x={result_recv[0]/1000:.6f}m, y={(result_recv[1]-AIRBRUSH_TO_ROBOT_CENTER_OFFSET)/1000:.6f}m")

print("\n" + "=" * 70)
print("ANALYSIS: Why is l1 different?")
print("=" * 70)

# The issue: sent delta vs received delta
l1_delta_sent = l1_sent - ROD1_init_L  # 400.5922 - 377 = 23.5922
l2_delta_sent = l2_sent - ROD2_init_L  # 304.8059 - 377 = -72.1941

l1_delta_recv = l1_recv - ROD1_init_L  # 398.39 - 377 = 21.39
l2_delta_recv = l2_recv - ROD2_init_L  # 304.82 - 377 = -72.18

print(f"Sent delta:     l1={l1_delta_sent:.4f}mm, l2={l2_delta_sent:.4f}mm")
print(f"Received delta: l1={l1_delta_recv:.2f}mm, l2={l2_delta_recv:.2f}mm")

# What int16 values would Arduino report?
l1_raw_expected = int(l1_delta_sent * 100)  # 2359
l2_raw_expected = int(l2_delta_sent * 100)  # -7219

l1_raw_actual = int(l1_delta_recv * 100)  # 2139
l2_raw_actual = int(l2_delta_recv * 100)  # -7218

print(f"\nExpected raw int16: l1={l1_raw_expected}, l2={l2_raw_expected}")
print(f"Actual raw int16:   l1={l1_raw_actual}, l2={l2_raw_actual}")
print(f"Difference in raw:  l1={l1_raw_actual - l1_raw_expected} ({(l1_raw_actual - l1_raw_expected)/100}mm)")
print(f"                    l2={l2_raw_actual - l2_raw_expected} ({(l2_raw_actual - l2_raw_expected)/100}mm)")
