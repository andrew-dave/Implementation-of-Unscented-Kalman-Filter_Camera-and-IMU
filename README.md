# Implementation of Unscented Kalman Filter_Vision+IMU
 The goal of this project is to implement sensor fusion using the Unscented Kalman filter by making use of the IMU data and the vision-based pose and velocities estimated. The vision-based position and orientation is obtained through projective transformation and the linear and angular velocity are calculated through optical flow. Since this system is highly non-linear, the Unscented Kalman filter captures the non-linearity of the model much better to estimate the state of the quadrotor moving in an experimental space. 
# System State:
![image](https://github.com/user-attachments/assets/dddc7fa0-a499-4f43-a570-df25f644a184)

Refer documentation for detailed explanation of implementation
# Execute this implementation:
 - Clone this repository and open the directory within the MATLAB environment. Run UKF_kalmanfilt_Part1.m to visualize outcomes of UKF using Camera for measurement(apriltag pose estimation)
 - Run the UKF_kalmanfilt_Part1.m to understand the outcomes of UKF using velocity estimated using optical flow and how the approach captures the non-linearity of the system.
