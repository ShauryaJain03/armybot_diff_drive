4 WHEEL DIFF DRIVE BOT - 

configured ros2_control, outputs command velocities on bot_controller/cmd_vel_unstamped

visulise urdf in rviz w/o launch file - 
ros2 launch urdf_tutorial display.launch.py model:=/home/shaurya/armybot_diff/src/bot_description/urdf/bot.urdf.xacro

launch gazebo sim in custom world - 
ros2 launch bot_description gazebo.launch.py world_name=test_new

launch keyboard teleop with ros2_control - 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bot_controller/cmd_vel_unstamped

computer vision node - 
ros2 run bot_vision vision_node

slam with slam_toolbox - 
ros2 launch slam_toolbox online_async_launch.py file:=./src/bot_description/config/mapper_params_online_async.yaml use_sim_time:=true