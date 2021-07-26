import numpy as np

str = '''<?xml version="1.0"?>
<robot name="LargeConeControl">

  <material name="gray">
      <color rgba="0.66 0.66 0.66 1"/>
  </material>


  <link name="base_link">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="1e-10" ixy="0." ixz="0." iyy="1e-10" iyz="0." izz="1e-10"/>
    </inertial>
  </link>

  <link name="apex_link_x">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="1e-10" ixy="0." ixz="0." iyy="1e-10" iyz="0." izz="1e-10"/>
    </inertial>
  </link>

  <link name="apex_link_y">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="1e-10" ixy="0." ixz="0." iyy="1e-10" iyz="0." izz="1e-10"/>
    </inertial>
  </link>

  <link name="apex_link_z">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="1e-10" ixy="0." ixz="0." iyy="1e-10" iyz="0." izz="1e-10"/>
    </inertial>
  </link>

  <link name="apex_link_dummy_1">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="1e-10" ixy="0." ixz="0." iyy="1e-10" iyz="0." izz="1e-10"/>
    </inertial>
  </link>

  <link name="apex_link_dummy_2">
    <inertial>
      <mass value="0.0001"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="1e-10" ixy="0." ixz="0." iyy="1e-10" iyz="0." izz="1e-10"/>
    </inertial>
  </link>

  <link name="cone">
    <inertial>
      <mass value="'''+str(1.5)+'''"/>
      <origin xyz="'''+str(0.0)+" "+str(0.262339)+" "+str(-1.109297)+'''"/>
      <inertia ixx="'''+str(0.021472)+'''" ixy="'''+str(0.)+'''" ixz="'''+str(0.)+'''" iyy="'''+str(0.020527)+'''" iyz="'''+str(0.003975)+'''" izz="'''+str(0.008467)+'''"/>
    </inertial>

    <contact>
      <lateral_friction value="0.4"/>
    </contact>

    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="mesh/large_cone_convex_apex_frame.obj"/>
      </geometry>
      <material name="gray"/>
    </visual>

    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="mesh/object_mesh.obj"/>
      </geometry>
    </collision>

  </link>


  <joint name="joint_apex_x" type="prismatic">
    <axis xyz="1 0 0"/>
    <parent link="base_link"/>
    <child link="apex_link_x"/>
    <limit lower="-100" upper="100"/>
  </joint>

  <joint name="joint_apex_y" type="prismatic">
    <axis xyz="0 1 0"/>
    <parent link="apex_link_x"/>
    <child link="apex_link_y"/>
    <limit lower="-100" upper="100"/>
  </joint>

  <joint name="joint_apex_z" type="prismatic">
    <axis xyz="0 0 1"/>
    <parent link="apex_link_y"/>
    <child link="apex_link_z"/>
    <limit lower="-100" upper="100"/>
  </joint>

  <joint name="joint_apex_dummy_1" type="spherical">
    <axis xyz="0 0 1"/>
    <parent link="apex_link_z"/>
    <child link="apex_link_dummy_1"/>
  </joint>

  <joint name="joint_apex_dummy_2" type="spherical">
    <axis xyz="0 1 0"/>
    <parent link="apex_link_dummy_1"/>
    <child link="apex_link_dummy_2"/>
  </joint>

  <joint name="joint_apex_dummy_3" type="spherical">
    <axis xyz="1 0 0"/>
    <parent link="apex_link_dummy_2"/>
    <child link="cone"/>
  </joint>

</robot>
'''

with open('myfile.urdf', 'w') as f:
    f.write(str)
