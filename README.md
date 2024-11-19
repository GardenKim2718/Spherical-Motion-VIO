# Spherical-Motion-VIO
HYU2024 Dept of Automotive Engineering Capstone Design Project
---------------------------------------------------------------
Visual Inertial Odometry project for estimating camera pose within Spherical Motion

Hardware Setting
----------------------
Platform: Jetson Nano (Ubuntu 20.04) 

IMU: MPU-6050 or MPU-9250(magnetometer function not supported)

Camera: IMX-219 (CSI cable connection)

Usage
----------------------
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
Action Matrix Solver is python varaition from Spherical SFM project(<https://github.com/jonathanventura/spherical-sfm>),

a research conducted by Pf.Jonathan Ventura



Special Thanks to Sungjin Park(<https://github.com/SuNy4>) for providing jetcam repository for operating IMX219 camera with Jetson Nano
