Step 3 — Plan for cameras and following

Cameras:
Add RGBD sensors to each robot SDF you copied in homework1_launcher/models/vehicle/model.sdf (or create vehicle_rgbd). You can reuse syntax from the camera demo SDFs in the repo. After adding sensors, run gz topic -l to find their Gazebo topics and extend your diff_drive.yaml with image/depth bridges.
RViz: add Image displays subscribed to your ROS topics.
Following:
Either reuse your C++ ball_tracker_node.cpp logic in a new dedicated package (recommended), or write a simpler Python follower as earlier suggested. I recommend a new package homework1_follow (so you don’t modify ros_gz_sim):
C++: depends on rclcpp, cv_bridge, OpenCV, sensor_msgs, geometry_msgs, nav_msgs.
Or Python: rclpy + Twist/Odometry.
If you want, I can scaffold homework1_follow and show the SDF camera blocks + bridge YAML once you confirm RGBD vs RGB+Depth preference.

What you can safely leave alone right now

homework1/ros_gz_sim_demos: don’t delete, don’t build. You’re duplicating only what’s needed into homework1_launcher.
homework1/ros_gz_sim, homework1/ros_gz_bridge: rely on system installs; build only if needed.
All other demo launchers/configs unrelated to diff_drive (listed above).
Next actions I can take

Add RGBD camera sensors to the copied robot SDFs and extend diff_drive.yaml with exact bridge topic pairs.
Create homework1_follow with a follower node and a combined launch that brings up sim + follower.