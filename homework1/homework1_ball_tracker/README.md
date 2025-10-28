# Ball Tracker Project

## Overview
The Ball Tracker project implements a ROS2 node that detects and tracks a ball in a camera stream using OpenCV. The node subscribes to image and depth topics, processes the images to identify the ball, and publishes its position. The project includes configurations for visualizing the tracking process in RViz.

## Project Structure
```
homework1_ball_tracker
├── src
│   └── camera_tracker_node.cpp         # Implementation of the CameraTrackerNode class
├── launch
│   ├── camera_tracker_rviz.launch.py   # Launch file for RViz visualization
│   └── camera_robot_tracker.launch.py  # Launch file for initializing the camera tracker node
├── rviz
│   └── camera_tracker.rviz              # RViz configuration for camera tracking visualization
├── config
│   └── params.yaml                    # Configuration parameters for the camera tracker node
├── CMakeLists.txt                     # CMake build configuration
├── package.xml                        # Package metadata
└── README.md                          # Project documentation
```

## Setup Instructions
1. **Install Dependencies**: Ensure that you have ROS2 and the necessary dependencies installed, including OpenCV and cv_bridge.

2. **Build the Project**:
   Navigate to the project directory and run:
   ```
   colcon build --symlink-install
   ```

3. **Source the Setup File**:
   After building, source the setup file:
   ```
   source install/setup.bash
   ```

4. **Launch the Ball Tracker**:
   To start the ball tracker node along with the camera stream and RViz visualization, use the following command:
   ```
   ros2 launch homework1_ball_tracker camera_ball_tracker.launch.py
   ```

5. **Visualize in RViz**:
   The RViz configuration will automatically load, displaying the camera feed and any detected ball markers.

## Usage Guidelines
- Adjust parameters in `config/params.yaml` to fine-tune the ball detection behavior, such as `min_area` for detection sensitivity and `ema_alpha` for smoothing the position estimates.
- Use the RViz interface to visualize the tracking process and monitor the ball's position in real-time.

## Additional Information
For more details on the implementation, refer to the source code in `src/ball_tracker_node.cpp`. The launch files in the `launch` directory provide the necessary configurations to run the ball tracker and visualize it in RViz.