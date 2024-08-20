import time
import serial
import tkinter as tk
from tkinter import ttk
import threading

# Global variables to share between threads
serial_port = '/dev/ttyACM0'  # Update with your serial port

baud_rate = 115200

def Check_Sum(data, count_number, mode):
    check_sum = 0
    for k in range(count_number):
        check_sum ^= data[k]
    return check_sum

def convert_to_signed(value):
    if value & 0x8000:
        return value - 0x10000
    return value

def receive_serial_data(ser, data_labels):
    while True:
        received_char = ser.read(1)
        print(str(received_char))
        if received_char == b'{':
            received_data = ser.read(27)
            
            received_data = received_char +received_data
            Frame_Header = received_data[0]
            Flag_Stop = received_data[1]

            X_speed = convert_to_signed((received_data[2] << 8) | received_data[3])
            Y_speed = convert_to_signed((received_data[4] << 8) | received_data[5])
            Z_speed = convert_to_signed((received_data[6] << 8) | received_data[7])

            X_accel = convert_to_signed((received_data[8] << 8) | received_data[9])
            Y_accel = convert_to_signed((received_data[10] << 8) | received_data[11])
            Z_accel = convert_to_signed((received_data[12] << 8) | received_data[13])

            X_gyro = convert_to_signed((received_data[14] << 8) | received_data[15])
            Y_gyro = convert_to_signed((received_data[16] << 8) | received_data[17])
            Z_gyro = convert_to_signed((received_data[18] << 8) | received_data[19])

            Power_Voltage = (received_data[20] << 8) | received_data[21]

            Stepper_X = convert_to_signed((received_data[22] << 8) | received_data[23])
            Stepper_Y = convert_to_signed((received_data[24] << 8) | received_data[25])

            Checksum = received_data[26]
            Frame_Tail = received_data[27]

            expected_checksum = Check_Sum(received_data, 26, 1)

            data = {
                'X Speed': f"{X_speed / 1000:.3f}",
                'Y Speed': f"{Y_speed / 1000:.3f}",
                'Z Speed': f"{Z_speed / 1000:.3f}",
                'X Acceleration': f"{X_accel / 1000:.3f}",
                'Y Acceleration': f"{Y_accel / 1000:.3f}",
                'Z Acceleration': f"{Z_accel / 1000:.3f}",
                'X Gyroscope': f"{X_gyro / 1000:.3f}",
                'Y Gyroscope': f"{Y_gyro / 1000:.3f}",
                'Z Gyroscope': f"{Z_gyro / 1000:.3f}",
                'Stepper X': f"{Stepper_X}",
                'Stepper Y': f"{Stepper_Y}",
                'Battery Voltage': f"{Power_Voltage / 1000:.3f}"
            }

            for key, (label, value_label) in data_labels.items():
                value_label.config(text=data[key])

def generate_command_bytes(velocity_x, velocity_y, angular_z, stepper1, stepper2, compressor1, compressor2):
    velocity_x = int(float(velocity_x) * 1000)
    velocity_y = int(float(velocity_y) * 1000)
    angular_z = int(float(angular_z) * 1000)
    stepper1 = int(stepper1)
    stepper2 = int(stepper2)
    compressor1 = int(compressor1)
    compressor2 = int(compressor2)

    command = [
        0x7B,  # FRAME_HEADER
        0x00,
        0x00,
        velocity_x >> 8 & 0xFF,
        velocity_x & 0xFF,
        velocity_y >> 8 & 0xFF,
        velocity_y & 0xFF,
        angular_z >> 8 & 0xFF,
        angular_z & 0xFF,
        stepper1 >> 8 & 0xFF,
        stepper1 & 0xFF,
        stepper2 >> 8 & 0xFF,
        stepper2 & 0xFF,
        compressor1 >> 8 & 0xFF,
        compressor1 & 0xFF,
        compressor2 >> 8 & 0xFF,
        compressor2 & 0xFF,
        0x00,  # Placeholder for checksum
        0x7D  # FRAME_TAIL
    ]

    checksum = Check_Sum(command, 17, 0)
    command[17] = checksum
    return command

def send_serial_data(ser, entries):
    values = [entry.get() for entry in entries.values()]
    command_bytes = generate_command_bytes(*values)
    for byte in command_bytes:
        ser.write(bytes([byte]))
        time.sleep(0.001)
    print("Sent command over serial")

def create_labeled_entry(parent, label_text, row):
    frame = tk.Frame(parent, bg='#2C3E50')
    frame.grid(row=row, column=0, sticky='ew', padx=10, pady=5)
    
    label = tk.Label(frame, text=label_text, bg='#2C3E50', fg='white', width=15, anchor='w')
    label.pack(side='left', padx=5)
    
    entry = tk.Entry(frame, bg='#ECF0F1', fg='#2C3E50', width=10)
    entry.pack(side='left', padx=5)
    
    return entry

def create_data_display(parent, label_text, row):
    frame = tk.Frame(parent, bg='#34495E')
    frame.grid(row=row, column=0, sticky='ew', padx=10, pady=5)
    
    label = tk.Label(frame, text=label_text, bg='#34495E', fg='#ECF0F1', width=15, anchor='w')
    label.pack(side='left', padx=5)
    
    value_label = tk.Label(frame, text="0.000", bg='#34495E', fg='#2ECC71', width=10, anchor='e')
    value_label.pack(side='left', padx=5)
    
    return label, value_label

if __name__ == "__main__":
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Serial port opened: {ser.is_open}")

    root = tk.Tk()
    root.title("Serial Control GUI")
    root.configure(bg='#2C3E50')
    root.geometry("300x750")  # Adjusted height to accommodate all elements

    style = ttk.Style()
    style.theme_use('clam')
    style.configure('TButton', background='#3498DB', foreground='white', font=('Arial', 10, 'bold'))

    # Frame for displaying received data
    data_frame = tk.Frame(root, bg='#2C3E50', padx=10, pady=10)
    data_frame.grid(row=0, column=0, sticky='nsew')

    data_labels = {}
    data_keys = [
        'X Speed', 'Y Speed', 'Z Speed',
        'X Acceleration', 'Y Acceleration', 'Z Acceleration',
        'X Gyroscope', 'Y Gyroscope', 'Z Gyroscope',
        'Stepper X', 'Stepper Y', 'Battery Voltage'
    ]

    for i, key in enumerate(data_keys):
        data_labels[key] = create_data_display(data_frame, key, i)

    # Frame for input fields
    input_frame = tk.Frame(root, bg='#2C3E50', padx=10, pady=10)
    input_frame.grid(row=1, column=0, sticky='nsew')

    entries = {}
    variables = ['velocity_x', 'velocity_y', 'angular_z', 'stepper1', 'stepper2', 'compressor1', 'compressor2']

    for i, var in enumerate(variables):
        entries[var] = create_labeled_entry(input_frame, var.capitalize(), i)

    # Send button
    send_button = ttk.Button(root, text="Send Command", command=lambda: send_serial_data(ser, entries), style='TButton')
    send_button.grid(row=2, column=0, pady=10)

    # Configure grid weights
    root.grid_rowconfigure(0, weight=3)
    root.grid_rowconfigure(1, weight=2)
    root.grid_columnconfigure(0, weight=1)

    # Start the receive thread
    receive_thread = threading.Thread(target=receive_serial_data, args=(ser, data_labels), daemon=True)
    receive_thread.start()

    root.mainloop()

    ser.close()