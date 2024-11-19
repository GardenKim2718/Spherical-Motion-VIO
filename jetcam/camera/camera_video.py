# Import necessary libraries
import cv2
from jetcam.csi_camera import CSICamera

# Initialize the CSI camera
camera = CSICamera(capture_width=1280, capture_height=720, downsample=4, capture_fps=60)

# Setup for video saving
output_filename = 'output.avi'  # Name of the file to save
fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec setting
fps = 20  # Frame rate setting
frame_width = 1280  # Frame width
frame_height = 720  # Frame height

# Initialize VideoWriter object
out = cv2.VideoWriter(output_filename, fourcc, fps, (frame_width, frame_height))

# Real-time video capture and save loop
while True:
    # Read an image (frame) from the camera
    image = camera.read()
    
    # Check if the image is valid
    if image is not None:
        # Save the current frame to the video file
        out.write(image)

        # Display the image in a window
        cv2.imshow('Camera Feed', image)

        # Exit the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Release the camera and destroy all OpenCV windows
camera.cap.release()
out.release()  # Release the VideoWriter object
cv2.destroyAllWindows()
