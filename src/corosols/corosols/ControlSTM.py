import time
import serial
import tkinter as tk
from tkinter import scrolledtext
import threading

# Global variables to share between threads
serial_port = '/dev/ttyACM0'  # Update with your serial port
baud_rate = 115200


def Check_Sum(data, count_number, mode):
    check_sum = 0
    # Validate the data for checksum calculation
    for k in range(count_number):
        if mode == 1:
            # Calculate checksum for sending data
            check_sum ^= data[k]  # XOR operation for sending data
        elif mode == 0:
            # Calculate checksum for received data
            check_sum ^= data[k]  # XOR operation for received data

    return check_sum

def receive_serial_data(ser, text_widget):
    while True:
            received_data = ser.read(24)
            Frame_Header = received_data[0]  # Frame header (byte)
            Flag_Stop = received_data[1]  # Stop flag (byte)

            X_speed = (received_data[2] << 8) | received_data[3] 
            if X_speed & 0x8000:
                X_speed = X_speed - 0x10000# X-axis speed (16-bit integer)
            Y_speed = (received_data[4] << 8) | received_data[5]  # Y-axis speed (16-bit integer)
            Z_speed = (received_data[6] << 8) | received_data[7]  # Z-axis speed (16-bit integer)

            X_accel = (received_data[8] << 8) | received_data[9]   # X-axis acceleration (16-bit integer)
            Y_accel = (received_data[10] << 8) | received_data[11]  # Y-axis acceleration (16-bit integer)
            Z_accel = (received_data[12] << 8) | received_data[13]  # Z-axis acceleration (16-bit integer)

            X_gyro = (received_data[14] << 8) | received_data[15]  # X-axis gyroscope (16-bit integer)
            Y_gyro = (received_data[16] << 8) | received_data[17]  # Y-axis gyroscope (16-bit integer)
            Z_gyro = (received_data[18] << 8) | received_data[19]  # Z-axis gyroscope (16-bit integer)

            Power_Voltage = (received_data[20] << 8) | received_data[21]  # Battery voltage (16-bit integer)

            Checksum = received_data[22]  # Checksum (byte)
            Frame_Tail = received_data[23]  # Frame tail (byte)

            # Calculate expected checksum based on received data
            expected_checksum = Check_Sum(received_data,22, 1)  # Assuming Check_Sum is a function defined elsewhere

            # Verify checksum
            """if Checksum == expected_checksum:
                print("Checksum is valid!")
            else:
                print("Checksum mismatch!")"""
            
             # Display or use reconstructed data
            data_to_display = f"""
            X_speed: {X_speed/1000}
            Y_speed: {Y_speed/1000}
            Z_speed: {Z_speed/1000}
            X_acceleration: {X_accel/1000}
            Y_acceleration: {Y_accel/1000}
            Z_acceleration: {Z_accel/1000}
            X_gyroscope: {X_gyro/1000}
            Y_gyroscope: {Y_gyro/1000}
            Battery Voltage: {Power_Voltage/1000}
            """
            text_widget.insert(tk.END, data_to_display)
            text_widget.see(tk.END)  # Scroll the text widget to the end
            
      
def generate_command_bytes(x, y, z, a):
    # Scale the velocity values by 1000 to match the expected format in the C++ code
    velocity_x = int(float(x) * 1000)
    velocity_y = int(float(y) * 1000)
    angular_z = int(float(z) * 1000)
    airbrush = int(a)
    if (airbrush == 1):
        airbrush = 1000
        print(airbrush)
        
    else: 
        airbrush = 2000
        print(airbrush)
    # Create the command bytes with header, reserved bytes, velocity components, checksum, and tail
    command = [
        0x7B,  # FRAME_HEADER
        0x00,  # Reserved
        0x00,  # Reserved
        velocity_x >> 8 & 0xFF,  # High byte of velocity_x
        velocity_x & 0xFF,       # Low byte of velocity_x
        velocity_y >> 8 & 0xFF,  # High byte of velocity_y
        velocity_y & 0xFF,       # Low byte of velocity_y
        angular_z >> 8 & 0xFF,   # High byte of angular_z
        angular_z & 0xFF,  
        airbrush >> 8 & 0xFF,   # High byte of angular_z
        airbrush & 0xFF,
        0x00,  # Placeholder for checksum (to be calculated)
        0x7D   # FRAME_TAIL
    ]
    
    checksum = Check_Sum(command,11, 0)  # Sum of bytes excluding FRAME_HEADER and checksum itself
    command[11] = checksum
    
    return command

def send_serial_data(ser, x_entry, y_entry, z_entry,airbrush_entry):
    # Wait for user input to update x, y, z values
    x = x_entry.get()
    y = y_entry.get()
    z = z_entry.get()
    a = airbrush_entry.get()
    command_bytes = generate_command_bytes(x, y, z, a)
    #print(['sent data',command_bytes],sep = ' ')
    # Send command bytes over serial
    for byte in command_bytes:
        ser.write(bytes([byte]))
        time.sleep(0.001) 
    print("Sent command over serial")
    
def send_text_data(ser, text_entry):
    # Get the text from the entry widget
    text = text_entry.get()
    # Send the text over serial
    ser.write(text.encode())
    print("Sent text over serial")
    
if __name__ == "__main__":
# Open the serial port
    airbrush_state = 0
    ser = serial.Serial(serial_port, baud_rate)
    print(ser.is_open)
    # Create the main window
    root = tk.Tk()
    root.configure(bg='lightblue')  # Change the background color of the window

    # Create a scrolled text widget for displaying received data
    received_data = scrolledtext.ScrolledText(root, width=40, height=10, bg='white', fg='black')
    received_data.pack(side=tk.LEFT)

    # Create entry widgets for entering x, y, z values
    airbrush_label = tk.Label(root, text="Airbrush Command:", bg='lightblue', fg='black')
    airbrush_label.pack(side=tk.LEFT)
    airbrush_entry = tk.Entry(root)
    airbrush_entry.pack(side=tk.LEFT)
   
    x_label = tk.Label(root, text="X:", bg='lightblue', fg='black')
    x_label.pack(side=tk.LEFT)
    x_entry = tk.Entry(root)
    x_entry.pack(side=tk.LEFT)
    
    y_label = tk.Label(root, text="Y:", bg='lightblue', fg='black') 
    y_label.pack(side=tk.LEFT)
    y_entry = tk.Entry(root)
    y_entry.pack(side=tk.LEFT)
    
    
    z_label = tk.Label(root, text="Z:", bg='lightblue', fg='black')
    z_label.pack(side=tk.LEFT)
    z_entry = tk.Entry(root)
    z_entry.pack(side=tk.LEFT)
    
    # Create a send button
    send_button = tk.Button(root, text="Send", command=lambda: send_serial_data(ser, x_entry, y_entry, z_entry, airbrush_entry), bg='green', fg='white')
    send_button.pack(side=tk.LEFT)

    text_entry = tk.Entry(root)
    text_entry.pack(side=tk.LEFT)

    # Create a send button for the text
    send_text_button = tk.Button(root, text="Send Text", command=lambda: send_text_data(ser, text_entry), bg='green', fg='white')
    send_text_button.pack(side=tk.LEFT)

    # Create and start the receive thread
    receive_thread = threading.Thread(target=receive_serial_data, args=(ser, received_data))
    receive_thread.start()

    # Start the Tkinter event loop
    root.mainloop()