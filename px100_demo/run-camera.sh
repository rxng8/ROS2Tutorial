source /opt/ros/noetic/setup.bash
source ~/interbotix_ws/devel/setup.bash
exec ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640,480,60