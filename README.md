# Color Based Navigation using ROS2 and OpenCV

## Overview

This project implements autonomous color-based navigation using ROS2, OpenCV, Gazebo and TurtleBot3.

The robot:

- detects a green object
- aligns itself using a proportional controller
- moves toward the object
- uses lidar to estimate distance
- stops near the object
- searches for the object if it leaves the frame

---

# Technologies Used

- ROS2 Humble
- Gazebo
- TurtleBot3
- OpenCV
- Python
- Lidar
- Computer Vision

---

# Features

- HSV color thresholding
- Contour detection
- Object center estimation
- Horizontal error calculation
- P-controller steering
- Search/recovery behavior
- Lidar-based stopping
- Real-time camera visualization

---

# Project Structure

```text
color-based-navigation-ros2/
│
├── README.md
├── LICENSE
│
├── screenshots/
│   ├── gazebo_world.png
│   ├── camera_feed.png
│   ├── green_mask.png
│   └── tracking.png
│
├── videos/
│   └── demo.mp4
│
├── green_sphere/
│   └── model.sdf
│
└── color_nav/
    │
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    │
    ├── launch/
    │   └── full_sim.launch.py
    │
    ├── resource/
    │   └── color_nav
    │
    └── color_nav/
        ├── __init__.py
        └── color_follower.py
```

---

# Setup Instructions

## 1. Install Dependencies

Make sure ROS2 Humble and TurtleBot3 packages are installed.

Install additional packages:

```bash
sudo apt update

sudo apt install \
ros-humble-cv-bridge \
ros-humble-gazebo-ros-pkgs \
ros-humble-turtlebot3 \
ros-humble-turtlebot3-gazebo \
python3-opencv
```

---

## 2. Create ROS2 Workspace

```bash
mkdir -p ~/ros2_ws/src
```

---

## 3. Clone Repository

```bash
cd ~/ros2_ws/src
git clone my_repository_link
```

---

## 4. Build Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

---

## 5. Source Workspace

```bash
source install/setup.bash
```

---

# Running the Simulation

## Terminal 1 — Launch Gazebo

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

---

## Terminal 2 — Spawn Green Sphere

```bash
ros2 run gazebo_ros spawn_entity.py \
-file ~/green_sphere/model.sdf \
-entity green_ball
```

---

## Terminal 3 — Run Color Navigation Node

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run color_nav color_follower
```

---

# Viewing Camera Feed

To view the camera stream:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select:

```text
/camera/image_raw
```

from the dropdown menu.

---

# Algorithm

## Step 1 — Camera Subscription

The robot subscribes to:

```text
/camera/image_raw
```

using ROS2 image messages.

---

## Step 2 — Convert Image to HSV

The incoming BGR image is converted into HSV color space.

HSV is preferred because it separates color information from brightness.

---

## Step 3 — Green Color Thresholding

A binary mask is created using:

```python
cv2.inRange()
```

Only green pixels are retained.

---

## Step 4 — Contour Detection

Contours are extracted from the mask.

The largest contour is assumed to be the target object.

---

## Step 5 — Object Center Calculation

The center of the object is computed:

```text
error = object_center_x - image_center_x
```

This horizontal error determines how far the object is from the image center.

---

## Step 6 — P Controller

A proportional controller rotates the robot toward the object.

```text
angular_velocity = -Kp × error
```

Behavior:

- large error → faster turning
- small error → slower turning
- zero error → object centered

---

## Step 7 — Forward Motion

The robot moves forward while continuously correcting steering.

---

## Step 8 — Object Recovery

If the object leaves the camera frame:

- contours disappear
- robot enters search mode
- robot rotates slowly
- scanning continues until object reappears

Recovery behavior:

```python
twist.angular.z = 0.2
twist.linear.x = 0.0
```

---

## Step 9 — Lidar-Based Stopping

Camera images alone cannot estimate distance reliably.

Therefore lidar data from:

```text
/scan
```

is used.

When the front obstacle distance becomes smaller than a threshold, the robot stops.

---



# Future Improvements

- We can implement PID controller completely for smoother working
- SLAM integration 
- Obstacle avoidance
- Path planning
- Multiple object tracking
- Deep learning-based detection

---

# Author

Nandini Taneja
