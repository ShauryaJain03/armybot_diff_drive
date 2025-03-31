4 Wheel Differential Drive QR Code Following Bot with LiDAR-based Obstacle Avoidance and Autonomous Return

visulise urdf in rviz w/o launch file - 
ros2 launch urdf_tutorial display.launch.py model:=/home/shaurya/armybot_diff/src/bot_description/urdf/bot.urdf.xacro

launch gazebo sim in custom world - 
ros2 launch bot_description gazebo.launch.py world_name={test_new/small_house/small_warehouse/empty/room_with_walls}

launch keyboard teleop with ros2_control - configured ros2_control, outputs command velocities on bot_controller/cmd_vel_unstamped
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/bot_controller/cmd_vel_unstamped

computer vision node - 
ros2 run bot_vision vision_node

slam with slam_toolbox - 
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/bot_description/config/mapper_params_online_async.yaml use_sim_time:=true

after running this , create a map by moving around in the environment. save map files both save and serialize. then close rviz and slam toolbox and change mapping to localization in the mapper_params_online_async.yaml file and set the path to saved map

re run both slam toolbox, rviz2, gazebo and set fixed frame to ODOM in rviz2 and subscribe to /map topic and also change durability policy under topic to transient local, 
this brings up our loaded map and we can proceed to localization by running - 

ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/bot_description/config/mapper_params_online_async.yaml use_sim_time:=true


twist_mux - 
ros2 run twist_mux twist_mux --ros-args --params-file ./src/bot_description/config/twist_mux.yaml -r cmd_vel_out:=bot_controller/cmd_vel_unstamped

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy

navigation using ros2 - 

ros2 launch bot_description navigation_launch.py use_sim_time:=true

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true




