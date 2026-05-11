# Visual Servoing and Target Tracking with TurtleBot3

[![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Foxy-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Python-3.8+-yellow.svg)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-Computer%20Vision-green.svg)](https://opencv.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Simulation-orange.svg)](https://gazebosim.org/home)

## 📌 Overview
This repository contains a ROS2 Python package demonstrating a closed-loop **Visual Servoing** system. The objective is to autonomously navigate a differential-drive robot (TurtleBot3 Waffle Pi) towards a specific colored target (a green sphere) using pure computer vision and proportional control logic, while dynamically handling edge cases such as target occlusion and proximity-based halting.

---

## 🧠 Technical Implementation & Problem Solving

The following details the methodology and technical solutions applied to satisfy the core requirements of the system architecture.

### A. Simulation Setup & Environment
* **Platform:** The standard `turtlebot3_gazebo` package was utilized. The `waffle_pi` model was explicitly exported (`export TURTLEBOT3_MODEL=waffle_pi`) as it features a simulated front-facing RGB camera by default, unlike the base `burger` model.
* **Target Rendering:** A green sphere was instantiated in the Gazebo world. To prevent rendering artifacts (such as alpha-channel transparency during active physics simulation), the sphere was defined with a strict opaque material parameter (RGBA: `0 1 0 1`) ensuring reliable visibility against the camera's near-clipping plane.

### B. ROS2 Camera Data Pipeline
* **Node Architecture:** A custom ROS2 Node (`CameraSubscriber`) was developed to bridge the simulated sensor output with the processing pipeline. 
* **Deserialization:** The node subscribes to the `sensor_msgs/msg/Image` over the `/camera/image_raw` topic. The `cv_bridge` library is leveraged to convert the ROS message payload into a standardized OpenCV NumPy array (`bgr8` encoding), making the data actionable for the computer vision pipeline.

### C. Vision Processing & Error Metric Calculation
To isolate the target and determine its relative position, a robust image processing pipeline was implemented:
1. **Color Space Transformation:** The raw BGR image is converted to the **HSV (Hue, Saturation, Value)** color space. This decouples chromaticity from illumination, making the color thresholding highly resilient to dynamic lighting conditions in the Gazebo physics engine.
2. **Thresholding & Masking:** `cv2.inRange` is applied to extract the target color, generating a binary mask.
3. **Contour Analysis:** `cv2.findContours` identifies the boundaries of the thresholded objects. The system dynamically filters for the largest contour area to reject background noise.
4. **Centroid Extraction:** Spatial image moments (`cv2.moments`) are calculated. The geometric centroid of the target is derived using $C_x = M_{10}/M_{00}$ and $C_y = M_{01}/M_{00}$.
5. **Error Calculation:** The horizontal spatial error is calculated as the difference between the centroid's X-coordinate and the image frame's center-point width (`error = cx - image_center_x`).

### D & E. Kinematics & Proportional Control
The robot is steered using a **Proportional (P) Controller** publishing `geometry_msgs/msg/Twist` messages to the `/cmd_vel` topic.
* **Rotational Kinematics (Yaw):** The angular velocity ($\omega_z$) is computed proportionally to the visual error: `twist.angular.z = -Kp * error`. A tuned gain ($K_p = 0.002$) ensures smooth asymptotic convergence towards the target without overshooting or aggressive oscillation.
* **Translational Kinematics (Surge):** Forward linear velocity ($v_x$) acts on a conditional deadband. If the absolute error falls below a tightly defined pixel threshold (`|error| < 30`), the target is considered "locked," and a constant forward velocity (`0.15 m/s`) is applied. Otherwise, linear velocity is halted (`0.0 m/s`) to prioritize rotational alignment.

### F. Target Loss Recovery (State Memory)
If the robot rotates too slowly or the target rapidly exits the camera's Field of View (FOV), the vision pipeline will return zero contours. 
* **Solution:** A short-term memory variable (`self.last_error`) caches the sign of the error metric every frame the target is visible. 
* **Recovery Behavior:** Upon target loss, the system triggers a state-machine fallback. It evaluates the cached error sign to determine whether the target exited the left or right side of the FOV, and applies an open-loop search velocity (`±0.4 rad/s`) in that direction until the target is reacquired.

### G. Proximity Detection & Halting
To prevent mechanical collision with the target, a stopping condition is required.
* **Implemented Vision Solution:** Utilizing `cv2.minEnclosingCircle`, the pixel radius of the target is continuously calculated. As the robot approaches, the radius increases due to perspective projection. A threshold is set (`radius > 120 pixels`); upon breach, linear velocity is clamped to zero.
* **Alternative Multi-modal Solutions (Theoretical Expansion):** While pure vision is effective, depth estimation via 2D vision is nonlinear. For production robustness, the system could fuse data from:
  1. **LiDAR (`/scan`):** Subscribing to `sensor_msgs/LaserScan` to check the `ranges[0]` index (directly in front of the robot) and stopping when distance `< 0.3m`.
  2. **RGB-D Camera (`/camera/depth/image_raw`):** Mapping the 2D visual centroid `(cx, cy)` to the corresponding 3D depth point cloud to trigger a hard stop based on actual spatial meters rather than pixel scaling.

---

## 🛠️ Prerequisites & Installation

* **OS:** Ubuntu 20.04 (Foxy) or 22.04 (Humble)
* **ROS2 Packages:** `rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`
* **External Dependencies:** `opencv-python`, `numpy`

```bash
# Clone into your ROS2 workspace
cd ~/turtlebot_ws/src
git clone [YOUR_REPO_URL]

# Build the package
cd ~/turtlebot_ws
colcon build --packages-select visual_servo_pkg
source install/setup.bash
