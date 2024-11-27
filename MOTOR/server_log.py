import socket
import threading
import time
import os

# Server settings
SERVER_IP = '127.0.0.1'
SERVER_PORT = 5005

# Initialize angles
angle1 = None
angle2 = None

# Initialize file by deleting contents at the start
def initialize_file():
    with open("angles_data.txt", "w") as file:
        file.write("")  # Open in write mode to clear content

def save_data(angle1, angle2):
    with open("angles_data.txt", "a") as file:  # Open file in append mode
        timestamp = time.time()  # Get the current timestamp
        log_entry = f"{timestamp} {angle1} {angle2} 0.0 \n"
        file.write(log_entry)  # Write to the file
        file.flush()  # Ensure data is written to the file
        print(log_entry.strip())  # Print to console (without newline)

def receive_data():
    global angle1, angle2
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((SERVER_IP, SERVER_PORT))
        print(f"Listening on {SERVER_IP}:{SERVER_PORT}...")
        
        while True:
            data, _ = sock.recvfrom(1024)
            message = data.decode()
            
            if "Motor 1 angle" in message:
                angle1 = float(message.split(":")[1].strip("° "))
                save_data(angle1, angle2)  # Save data when Motor 1 is updated

            elif "Motor 2 angle" in message:
                angle2 = float(message.split(":")[1].strip("° "))
                save_data(angle1, angle2)  # Save data when Motor 2 is updated

def main():
    # Initialize file to clear previous contents
    initialize_file()
    
    # Start data receiving thread
    receive_thread = threading.Thread(target=receive_data)
    receive_thread.start()

    # Keep the main thread alive
    receive_thread.join()

if __name__ == "__main__":
    main()
