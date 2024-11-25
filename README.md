# Spherical-Motion-VIO
HYU2024 Department of Automotive Engineering Capstone Design Project
---------------------------------------------------------------
Visual Inertial Odometry project for estimating camera pose within Spherical Motion

Hardware Setting
----------------------
Platform: Jetson Nano (Ubuntu 20.04) 

IMU: MPU-6050 or MPU-9250(magnetometer function not supported, gyro & accelerometer registers should be identical)

Camera: IMX-219 (CSI cable connection)

Environment
----------------------
Ubuntu 20.04

OpenCV Version: 4.5.3

Usage
----------------------
As the IMU and VIO operate in different frequencies, the IMU code and VIO code had to be run separately.

The IMU code will send data via socket communication to the VIO code.

1. VINS (baseline VIO without regarding Spherical Motion)

   ```
   python3 comparison_visual_VIO.py
   python3 INS.py
   ```
   
3. Action Matrix Solver applied VIO

   Edit the visual odometry dependency within realtime_VIO.py to realtime_VO_decomp

   ```
   from realtime_VO_decomp import VisualOdometry, PinholeCamera
   ```

   ```
   python3 AHRS.py
   python3 realtime_VIO.py
   ```
   
5. SM-VIO (final model)

   Edit the visual odometry dependency within realtime_VIO.py to realtime_VO_org
   
   ```
   from realtime_VO_org import VisualOdometry, PinholeCamera
   ```
   
   ```
   python3 AHRS.py
   python3 realtime_VIO.py
   ```
7. AHRS only
   
   ```
   python3 AHRS_only.py
   ```

Contributions
----------------------
Action Matrix Solver is python varaition of Spherical SFM project(<https://github.com/jonathanventura/spherical-sfm>),

a research conducted by Pf. Ventura



Special thanks to Sungjin Park(<https://github.com/SuNy4>) for providing jetcam repository (<https://github.com/IRCVLab/HYU-2024-Embedded>) for operating IMX219 camera with Jetson Nano

Finally, we are grateful for all the help Pf. Hwang (<https://soonminhwang.github.io/>) gave us as advisory professor of this project.
