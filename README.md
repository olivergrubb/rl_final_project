# rl_final_project
Repository for final project for IR

Steps to install

- Clone the turtlebot3 package from here: https://github.com/ROBOTIS-GIT/turtlebot3 into your ros_ws/src directory  (Ensure you clone the foxy branch)
- Clone this repo into your ros_ws/src directory
- Navigate to ros_ws directory
- Run the following commands:

  - source /opt/ros/foxy/setup.bash
  - rosdep install --from-paths src -r -y
  - colcon build --symlink-install
  (Open new terminal)
  - export TURTLEBOT3_MODEL=waffle
  - source install/setup.bash
  - ros2 launch turtlebot3_multi_robot gazebo_multi_world.launch.py 

If all works correctly gazebo should launch with the two robots and all relevant topics should exist within the correct namespaces.
