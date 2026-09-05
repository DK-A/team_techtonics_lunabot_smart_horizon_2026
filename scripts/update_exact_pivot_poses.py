import os

sdf_content = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="lunabot">
    <pose>0 0 0.0 0 0 0</pose>

    <!-- BASE LINK (Chassis Body & Avionics Frame) -->
    <link name="base_link">
      <pose>0 0 0.41 0 0 0</pose>
      <inertial>
        <mass>60.0</mass>
        <pose>0 0 -0.14 0 0 0</pose>
        <inertia>
          <ixx>2.164</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>3.054</iyy><iyz>0.0</iyz>
          <izz>4.734</izz>
        </inertia>
      </inertial>

      <visual name="chassis_visual">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://lunabot/meshes/chassis.glb</uri>
          </mesh>
        </geometry>
      </visual>

      <collision name="chassis_collision">
        <pose>0 0 0.0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.80 0.60 0.25</size>
          </box>
        </geometry>
      </collision>
    </link>

    <!-- PASSIVE DIFFERENTIAL BAR -->
    <link name="differential_bar">
      <pose relative_to="base_link">0.000 0.000 -0.110 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.105</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.001</iyy><iyz>0.0</iyz>
          <izz>0.105</izz>
        </inertia>
      </inertial>
      <visual name="differential_bar_visual">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://lunabot/meshes/differential_bar.glb</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="differential_bar_collision">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>0.04 0.60 0.035</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="differential_bar_joint" type="revolute">
      <parent>base_link</parent>
      <child>differential_bar</child>
      <pose relative_to="differential_bar">0 0 0 0 0 0</pose>
      <axis>
        <xyz>1 0 0</xyz>
        <limit>
          <lower>-0.15</lower>
          <upper>0.15</upper>
        </limit>
        <dynamics>
          <damping>1.0</damping>
          <friction>0.1</friction>
        </dynamics>
      </axis>
    </joint>

    <!-- ROCKER ARMS (LEFT & RIGHT) -->
    <link name="left_rocker">
      <pose relative_to="base_link">0.000 0.380 -0.110 0 0 0</pose>
      <inertial>
        <mass>6.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.005</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.250</iyy><iyz>0.0</iyz>
          <izz>0.250</izz>
        </inertia>
      </inertial>
      <visual name="left_rocker_visual">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://lunabot/meshes/left_rocker.glb</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="left_rocker_collision">
        <pose>0.30 0 -0.25 0 0 0</pose>
        <geometry>
          <box>
            <size>0.70 0.04 0.08</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="left_rocker_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_rocker</child>
      <pose relative_to="left_rocker">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.15</lower>
          <upper>0.15</upper>
        </limit>
        <dynamics>
          <damping>2.0</damping>
          <friction>0.1</friction>
          <spring_stiffness>0.0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>

    <link name="right_rocker">
      <pose relative_to="base_link">0.000 -0.380 -0.110 0 0 0</pose>
      <inertial>
        <mass>6.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.005</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.250</iyy><iyz>0.0</iyz>
          <izz>0.250</izz>
        </inertia>
      </inertial>
      <visual name="right_rocker_visual">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://lunabot/meshes/right_rocker.glb</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="right_rocker_collision">
        <pose>0.30 0 -0.25 0 0 0</pose>
        <geometry>
          <box>
            <size>0.70 0.04 0.08</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="right_rocker_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_rocker</child>
      <pose relative_to="right_rocker">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.15</lower>
          <upper>0.15</upper>
        </limit>
        <dynamics>
          <damping>2.0</damping>
          <friction>0.1</friction>
          <spring_stiffness>0.0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>

    <!-- BOGIE ARMS (LEFT & RIGHT) -->
    <link name="left_bogie">
      <pose relative_to="left_rocker">0.300 0.040 -0.350 0 0 0</pose>
      <inertial>
        <mass>4.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.003</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.100</iyy><iyz>0.0</iyz>
          <izz>0.100</izz>
        </inertia>
      </inertial>
      <visual name="left_bogie_visual">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://lunabot/meshes/left_bogie.glb</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="left_bogie_collision">
        <pose>-0.30 0 -0.05 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 0.04 0.06</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="left_bogie_joint" type="revolute">
      <parent>left_rocker</parent>
      <child>left_bogie</child>
      <pose relative_to="left_bogie">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.25</lower>
          <upper>0.25</upper>
        </limit>
        <dynamics>
          <damping>0.8</damping>
          <friction>0.05</friction>
          <spring_stiffness>0.0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>

    <link name="right_bogie">
      <pose relative_to="right_rocker">0.300 -0.040 -0.350 0 0 0</pose>
      <inertial>
        <mass>4.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.003</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.100</iyy><iyz>0.0</iyz>
          <izz>0.100</izz>
        </inertia>
      </inertial>
      <visual name="right_bogie_visual">
        <cast_shadows>true</cast_shadows>
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://lunabot/meshes/right_bogie.glb</uri>
          </mesh>
        </geometry>
      </visual>
      <collision name="right_bogie_collision">
        <pose>-0.30 0 -0.05 0 0 0</pose>
        <geometry>
          <box>
            <size>0.55 0.04 0.06</size>
          </box>
        </geometry>
      </collision>
    </link>

    <joint name="right_bogie_joint" type="revolute">
      <parent>right_rocker</parent>
      <child>right_bogie</child>
      <pose relative_to="right_bogie">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-0.25</lower>
          <upper>0.25</upper>
        </limit>
        <dynamics>
          <damping>0.8</damping>
          <friction>0.05</friction>
          <spring_stiffness>0.0</spring_stiffness>
        </dynamics>
      </axis>
    </joint>

    <!-- 6 WHEELS -->
    <link name="left_front_wheel">
      <pose relative_to="left_rocker">0.650 0.090 -0.525 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.028</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.050</iyy><iyz>0.0</iyz>
          <izz>0.028</izz>
        </inertia>
      </inertial>
      <visual name="left_front_wheel_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh><uri>model://lunabot/meshes/left_front_wheel.glb</uri></mesh>
        </geometry>
      </visual>
      <collision name="left_front_wheel_collision">
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.1825</radius>
            <length>0.1600</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="left_front_wheel_joint" type="revolute">
      <parent>left_rocker</parent>
      <child>left_front_wheel</child>
      <pose relative_to="left_front_wheel">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.01</damping><friction>0.01</friction></dynamics>
      </axis>
    </joint>

    <link name="left_middle_wheel">
      <pose relative_to="left_bogie">-0.300 0.050 -0.175 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.028</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.050</iyy><iyz>0.0</iyz>
          <izz>0.028</izz>
        </inertia>
      </inertial>
      <visual name="left_middle_wheel_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh><uri>model://lunabot/meshes/left_middle_wheel.glb</uri></mesh>
        </geometry>
      </visual>
      <collision name="left_middle_wheel_collision">
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.1825</radius>
            <length>0.1600</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="left_middle_wheel_joint" type="revolute">
      <parent>left_bogie</parent>
      <child>left_middle_wheel</child>
      <pose relative_to="left_middle_wheel">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.01</damping><friction>0.01</friction></dynamics>
      </axis>
    </joint>

    <link name="left_rear_wheel">
      <pose relative_to="left_bogie">-0.950 0.050 -0.175 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.028</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.050</iyy><iyz>0.0</iyz>
          <izz>0.028</izz>
        </inertia>
      </inertial>
      <visual name="left_rear_wheel_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh><uri>model://lunabot/meshes/left_rear_wheel.glb</uri></mesh>
        </geometry>
      </visual>
      <collision name="left_rear_wheel_collision">
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.1825</radius>
            <length>0.1600</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="left_rear_wheel_joint" type="revolute">
      <parent>left_bogie</parent>
      <child>left_rear_wheel</child>
      <pose relative_to="left_rear_wheel">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.01</damping><friction>0.01</friction></dynamics>
      </axis>
    </joint>

    <link name="right_front_wheel">
      <pose relative_to="right_rocker">0.650 -0.090 -0.525 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.028</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.050</iyy><iyz>0.0</iyz>
          <izz>0.028</izz>
        </inertia>
      </inertial>
      <visual name="right_front_wheel_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh><uri>model://lunabot/meshes/right_front_wheel.glb</uri></mesh>
        </geometry>
      </visual>
      <collision name="right_front_wheel_collision">
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.1825</radius>
            <length>0.1600</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="right_front_wheel_joint" type="revolute">
      <parent>right_rocker</parent>
      <child>right_front_wheel</child>
      <pose relative_to="right_front_wheel">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.01</damping><friction>0.01</friction></dynamics>
      </axis>
    </joint>

    <link name="right_middle_wheel">
      <pose relative_to="right_bogie">-0.300 -0.050 -0.175 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.028</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.050</iyy><iyz>0.0</iyz>
          <izz>0.028</izz>
        </inertia>
      </inertial>
      <visual name="right_middle_wheel_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh><uri>model://lunabot/meshes/right_middle_wheel.glb</uri></mesh>
        </geometry>
      </visual>
      <collision name="right_middle_wheel_collision">
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.1825</radius>
            <length>0.1600</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="right_middle_wheel_joint" type="revolute">
      <parent>right_bogie</parent>
      <child>right_middle_wheel</child>
      <pose relative_to="right_middle_wheel">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.01</damping><friction>0.01</friction></dynamics>
      </axis>
    </joint>

    <link name="right_rear_wheel">
      <pose relative_to="right_bogie">-0.950 -0.050 -0.175 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>
          <ixx>0.028</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
          <iyy>0.050</iyy><iyz>0.0</iyz>
          <izz>0.028</izz>
        </inertia>
      </inertial>
      <visual name="right_rear_wheel_visual">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <mesh><uri>model://lunabot/meshes/right_rear_wheel.glb</uri></mesh>
        </geometry>
      </visual>
      <collision name="right_rear_wheel_collision">
        <pose>0 0 0 1.57079632679 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.1825</radius>
            <length>0.1600</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <joint name="right_rear_wheel_joint" type="revolute">
      <parent>right_bogie</parent>
      <child>right_rear_wheel</child>
      <pose relative_to="right_rear_wheel">0 0 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.01</damping><friction>0.01</friction></dynamics>
      </axis>
    </joint>

    <!-- SENSORS -->
    <link name="lidar_link">
      <pose relative_to="base_link">0 0 0.315 0 0 0</pose>
      <inertial>
        <mass>0.8</mass>
        <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia>
      </inertial>
      <sensor name="gpu_lidar" type="gpu_lidar">
        <update_rate>10.0</update_rate>
        <topic>/scan</topic>
        <ray>
          <scan>
            <horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
          </scan>
          <range><min>0.15</min><max>25.0</max><resolution>0.01</resolution></range>
        </ray>
      </sensor>
    </link>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>

    <link name="imu_link">
      <pose relative_to="base_link">0.08 0 0.132 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia>
      </inertial>
      <sensor name="imu_sensor" type="imu">
        <update_rate>50.0</update_rate>
        <topic>/imu/data</topic>
      </sensor>
    </link>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
    </joint>

    <!-- GAZEBO PLUGINS -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_front_wheel_joint</left_joint>
      <left_joint>left_middle_wheel_joint</left_joint>
      <left_joint>left_rear_wheel_joint</left_joint>
      <right_joint>right_front_wheel_joint</right_joint>
      <right_joint>right_middle_wheel_joint</right_joint>
      <right_joint>right_rear_wheel_joint</right_joint>
      <wheel_separation>0.94</wheel_separation>
      <wheel_radius>0.1825</wheel_radius>
      <max_linear_acceleration>1.0</max_linear_acceleration>
      <min_linear_acceleration>-1.0</min_linear_acceleration>
      <topic>/cmd_vel</topic>
      <odom_topic>/odom</odom_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_link</child_frame_id>
    </plugin>
  </model>
</sdf>
"""

urdf_content = """<?xml version="1.0" ?>
<robot name="lunabot">
  <link name="base_link">
    <inertial>
      <mass value="60.0"/>
      <origin xyz="0 0 -0.14" rpy="0 0 0"/>
      <inertia ixx="2.164" ixy="0" ixz="0" iyy="3.054" iyz="0" izz="4.734"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="package://lunabot/meshes/chassis.glb"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.0" rpy="0 0 0"/>
      <geometry><box size="0.80 0.60 0.25"/></geometry>
    </collision>
  </link>

  <link name="left_rocker">
    <inertial><mass value="6.0"/><inertia ixx="0.005" ixy="0" ixz="0" iyy="0.25" iyz="0" izz="0.25"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/left_rocker.glb"/></geometry></visual>
    <collision><origin xyz="0.30 0 -0.25" rpy="0 0 0"/><geometry><box size="0.70 0.04 0.08"/></geometry></collision>
  </link>

  <joint name="left_rocker_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_rocker"/>
    <origin xyz="0.000 0.380 -0.110" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.15" upper="0.15" effort="100" velocity="1.0"/>
  </joint>

  <link name="right_rocker">
    <inertial><mass value="6.0"/><inertia ixx="0.005" ixy="0" ixz="0" iyy="0.25" iyz="0" izz="0.25"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/right_rocker.glb"/></geometry></visual>
    <collision><origin xyz="0.30 0 -0.25" rpy="0 0 0"/><geometry><box size="0.70 0.04 0.08"/></geometry></collision>
  </link>

  <joint name="right_rocker_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_rocker"/>
    <origin xyz="0.000 -0.380 -0.110" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.15" upper="0.15" effort="100" velocity="1.0"/>
  </joint>

  <link name="left_bogie">
    <inertial><mass value="4.0"/><inertia ixx="0.003" ixy="0" ixz="0" iyy="0.10" iyz="0" izz="0.10"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/left_bogie.glb"/></geometry></visual>
    <collision><origin xyz="-0.30 0 -0.05" rpy="0 0 0"/><geometry><box size="0.55 0.04 0.06"/></geometry></collision>
  </link>

  <joint name="left_bogie_joint" type="revolute">
    <parent link="left_rocker"/>
    <child link="left_bogie"/>
    <origin xyz="0.300 0.040 -0.350" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.25" upper="0.25" effort="50" velocity="1.0"/>
  </joint>

  <link name="right_bogie">
    <inertial><mass value="4.0"/><inertia ixx="0.003" ixy="0" ixz="0" iyy="0.10" iyz="0" izz="0.10"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/right_bogie.glb"/></geometry></visual>
    <collision><origin xyz="-0.30 0 -0.05" rpy="0 0 0"/><geometry><box size="0.55 0.04 0.06"/></geometry></collision>
  </link>

  <joint name="right_bogie_joint" type="revolute">
    <parent link="right_rocker"/>
    <child link="right_bogie"/>
    <origin xyz="0.300 -0.040 -0.350" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.25" upper="0.25" effort="50" velocity="1.0"/>
  </joint>

  <!-- 6 WHEELS -->
  <link name="left_front_wheel">
    <inertial><mass value="3.0"/><inertia ixx="0.028" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.028"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/left_front_wheel.glb"/></geometry></visual>
    <collision><origin xyz="0 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.1825" length="0.1600"/></geometry></collision>
  </link>
  <joint name="left_front_wheel_joint" type="continuous">
    <parent link="left_rocker"/>
    <child link="left_front_wheel"/>
    <origin xyz="0.650 0.090 -0.525" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="left_middle_wheel">
    <inertial><mass value="3.0"/><inertia ixx="0.028" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.028"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/left_middle_wheel.glb"/></geometry></visual>
    <collision><origin xyz="0 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.1825" length="0.1600"/></geometry></collision>
  </link>
  <joint name="left_middle_wheel_joint" type="continuous">
    <parent link="left_bogie"/>
    <child link="left_middle_wheel"/>
    <origin xyz="-0.300 0.050 -0.175" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="left_rear_wheel">
    <inertial><mass value="3.0"/><inertia ixx="0.028" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.028"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/left_rear_wheel.glb"/></geometry></visual>
    <collision><origin xyz="0 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.1825" length="0.1600"/></geometry></collision>
  </link>
  <joint name="left_rear_wheel_joint" type="continuous">
    <parent link="left_bogie"/>
    <child link="left_rear_wheel"/>
    <origin xyz="-0.950 0.050 -0.175" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="right_front_wheel">
    <inertial><mass value="3.0"/><inertia ixx="0.028" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.028"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/right_front_wheel.glb"/></geometry></visual>
    <collision><origin xyz="0 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.1825" length="0.1600"/></geometry></collision>
  </link>
  <joint name="right_front_wheel_joint" type="continuous">
    <parent link="right_rocker"/>
    <child link="right_front_wheel"/>
    <origin xyz="0.650 -0.090 -0.525" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="right_middle_wheel">
    <inertial><mass value="3.0"/><inertia ixx="0.028" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.028"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/right_middle_wheel.glb"/></geometry></visual>
    <collision><origin xyz="0 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.1825" length="0.1600"/></geometry></collision>
  </link>
  <joint name="right_middle_wheel_joint" type="continuous">
    <parent link="right_bogie"/>
    <child link="right_middle_wheel"/>
    <origin xyz="-0.300 -0.050 -0.175" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="right_rear_wheel">
    <inertial><mass value="3.0"/><inertia ixx="0.028" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.028"/></inertial>
    <visual><geometry><mesh filename="package://lunabot/meshes/right_rear_wheel.glb"/></geometry></visual>
    <collision><origin xyz="0 0 0" rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.1825" length="0.1600"/></geometry></collision>
  </link>
  <joint name="right_rear_wheel_joint" type="continuous">
    <parent link="right_bogie"/>
    <child link="right_rear_wheel"/>
    <origin xyz="-0.950 -0.050 -0.175" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
"""

with open('/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/model.sdf', 'w') as f:
    f.write(sdf_content)

with open('/home/dk05/Desktop/SMART_HORIZON/LUNA_PRO/environment/models/lunabot/lunabot.urdf', 'w') as f:
    f.write(urdf_content)

print("✅ UPDATED model.sdf AND lunabot.urdf WITH PERFECT PIVOT POSES FROM LB.BLEND!")
