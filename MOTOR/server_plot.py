import socket
import threading
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Server settings
SERVER_IP = '127.0.0.1'
SERVER_PORT = 5005

# Initialize angles
angle1 = None
angle2 = None

def angle_to_coordinates(angle1, angle2):
    x_coord = np.cos(np.radians(angle1)) * np.cos(np.radians(angle2))
    y_coord = np.sin(np.radians(angle1)) * np.cos(np.radians(angle2))
    z_coord = np.sin(np.radians(angle2))
    return x_coord, y_coord, z_coord

def receive_data():
    global angle1, angle2
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((SERVER_IP, SERVER_PORT))
        print(f"Listening on {SERVER_IP}:{SERVER_PORT}...")
        
        while True:
            data, _ = sock.recvfrom(1024)
            message = data.decode()
            print(f"Received: {message}")
            
            if "Motor 1 angle" in message:
                angle1 = float(message.split(":")[1].strip("° "))
            elif "Motor 2 angle" in message:
                angle2 = float(message.split(":")[1].strip("° "))

def update_plot(frame, point, line):
    global angle1, angle2
    # Update only if both angles are available
    if angle1 is not None and angle2 is not None:
        x, y, z = angle_to_coordinates(angle1, angle2)
        
        # Update point position
        point.set_data([x], [y])
        point.set_3d_properties([z])

        # Update line position
        line.set_data([0, x], [0, y])
        line.set_3d_properties([0, z])

def main():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])

    # Initialize point and line
    point, = ax.plot([0], [0], [0], 'bo')  # Blue point
    line, = ax.plot([0, 0], [0, 0], [0, 0], 'r')  # Red line

    # Start data receiving thread
    receive_thread = threading.Thread(target=receive_data)
    receive_thread.start()

    # Use FuncAnimation to update plot periodically
    ani = FuncAnimation(fig, update_plot, fargs=(point, line), interval=100)

    plt.show()

if __name__ == "__main__":
    main()
