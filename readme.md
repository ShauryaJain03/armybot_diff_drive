4 WHEEL DIFF DRIVE BOT - 

visulise urdf in rviz w/o launch file - 
ros2 launch urdf_tutorial display.launch.py model:=/home/shaurya/armybot_diff/src/bot_description/urdf/bot.urdf.xacro

launch gazebo sim - 
ros2 launch bot_description gazebo.launch.py

launch keyboard teleop - 
ros2 run teleop_twist_keyboard teleop_twist_keyboard

computer vision node - 
ros2 run bot_vision vision_node


<link name="rgb_camera">
    <inertial>
            <mass value="0.5" />
            <inertia ixx="0.01" ixy="0.0" ixz="0" 
            iyy="0.01" iyz="0" izz="0.01" />
    </inertial>

    <visual>
      
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>    <origin xyz="-0.35 0 0.15" rpy="0 0.0 3.14"/>


    <collision>
      <geometry>
          <box size="0.1 0.5 0.2"/>
      </geometry>
    </collision>
  </link>

  <joint name="camera_joint" type="fixed">
    <origin xyz="-0.43 0 0.0" rpy="0 0.0 3.14"/>
    <parent link="chassis"/>
    <child link="rgb_camera"/>
    <axis xyz="0.0 0 0"/>

  </joint>


    <gazebo reference="rgb_camera">
        <sensor name="camera" type="camera">
            <always_on>true</always_on>
            <update_rate>30.0</update_rate>
            <visualize>true</visualize>
            <topic>/camera/image_raw</topic>
            <gz_frame_id>/rgb_camera</gz_frame_id>
            <camera name="camera">
            <horizontal_fov>1.21126</horizontal_fov>
            <image>
                <width>640</width>
                <height>640</height>
                <format>R8G8B8</format>
            </image>
            <distortion>
                <k1>0.0</k1>
                <k2>0.0</k2>
                <k3>0.0</k3>
                <p1>0.0</p1>
                <p2>0.0</p2>
                <center>0.5 0.5</center>
            </distortion>
            </camera>
        </sensor>   
    </gazebo>


  <link name="laser_link">
    <inertial>
            <mass value="0.5" />
            <inertia ixx="0.01" ixy="0.0" ixz="0" 
            iyy="0.01" iyz="0" izz="0.01" />
    </inertial>

    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0  1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="laser_joint" type="fixed">
    <origin xyz="0 0 0.15" rpy="0 0.0 3.14"/>
    <parent link="chassis"/>
    <child link="laser_link"/>
    <axis xyz="0.0 0.0 1.0"/>

  </joint>

  <gazebo reference="laser_link">
    <sensor name="lidar" type="gpu_lidar">
      <always_on>true</always_on>
      <visualize>true</visualize>
      <topic>scan</topic>
      <update_rate>5</update_rate>
      <gz_frame_id>laser_link</gz_frame_id>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1.00000</resolution>
            <min_angle>0.000000</min_angle>
            <max_angle>6.280000</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.120000</min>
          <max>12.0</max>
          <resolution>0.02</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </lidar>
    </sensor>
  </gazebo>


    <gazebo reference="wheel_front_right">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <kp>500000.0</kp>
      <kd>10.0</kd>
      <minDepth>0.001</minDepth>
      <maxVel>0.1</maxVel>
      <fdir1>1 0 0</fdir1>
    </gazebo>

    <gazebo reference="wheel_front_left">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <kp>500000.0</kp>
      <kd>10.0</kd>
      <minDepth>0.001</minDepth>
      <maxVel>0.1</maxVel>
      <fdir1>1 0 0</fdir1>
    </gazebo>

    <gazebo reference="wheel_back_right">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <kp>500000.0</kp>
      <kd>10.0</kd>
      <minDepth>0.001</minDepth>
      <maxVel>0.1</maxVel>
      <fdir1>1 0 0</fdir1>
    </gazebo>

    <gazebo reference="wheel_back_left">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <kp>500000.0</kp>
      <kd>10.0</kd>
      <minDepth>0.001</minDepth>
      <maxVel>0.1</maxVel>
      <fdir1>1 0 0</fdir1>
    </gazebo>

    <gazebo>
        <plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">
              <parameters>
              </parameters>
        </plugin>
        <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"></plugin>
        <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
        <render_engine>ogre2</render_engine>
      </plugin>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <update_rate>100</update_rate>
    <left_joint>wheel_front_left_joint wheel_back_left_joint</left_joint>
    <right_joint>wheel_front_right_joint wheel_back_right_joint</right_joint>
    <wheel_separation>0.8</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>
    <max_wheel_torque>20</max_wheel_torque>
    <command_topic>cmd_vel</command_topic>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>false</publish_wheel_tf>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <odometry_source>world</odometry_source>
    <ros>
      <namespace>/</namespace>
    </ros>
  </plugin>
      
    </gazebo>
