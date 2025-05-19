FOLLOW THESE STEPS IN YOUR TERMINAL (linux)

cd ./py_send_camera_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select py_send_camera
. install/setup.bash
ros2 run py_send_camera send_camera