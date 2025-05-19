FOLLOW THESE STEPS IN YOUR TERMINAL (linux)

cd pyqt_rcv_cam_ihm_ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select pyqt_rcv_cam_ihm_ros2
. install/setup.bash
ros2 run pyqt_rcv_cam_ihm_ros2 rcv_cam_ihm