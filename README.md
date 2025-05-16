# ros2-gz-sim

colcon build && source install/setup.bash

ros2 launch gazebo_simulation launch_sim.launch.py

ros2 launch gazebo_simulation launch_sim_multi.launch.py

os2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/r1