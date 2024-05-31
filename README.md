
# Delto Gripper

<center><img src="https://cdn.imweb.me/thumbnail/20240102/cc620fcfefe42.png" style="width: 80%;"/></center>

<center><img src="https://cdn.imweb.me/thumbnail/20240102/4404e0bb0eae6.png" style="width: 80%;"/></center>

This is Delto Gripper ROS package.

Some of the new features are enabled by ROS and include 
decreased latency, improved security, and more flexibility regarding middleware configuration. 


Check also [Delto 3F Gripper Manual](https://www.tesollo.com/Community/?q=YToyOntzOjEyOiJrZXl3b3JkX3R5cGUiO3M6MzoiYWxsIjtzOjQ6InBhZ2UiO2k6MTt9&bmode=view&idx=18137982&t=board)


Check also [Delto 2F Gripper Manual](https://www.tesollo.com/Community/?q=YToxOntzOjEyOiJrZXl3b3JkX3R5cGUiO3M6MzoiYWxsIjt9&bmode=view&idx=18137991&t=board)



## Build Status

<table width="100%">
  <tr>
    <th>ROS Distro</th>
    <th>ROS1 (18.04)</th>
  </tr>
  <tr>
   <th> Branch </th>
   <th>  Melodic </th> 
  </tr>
</table>




## Packages in the Repository:

  - `delto_3f_description` - URDF, mesh file

  - `delto_3f_driver` - Delto Gripper 3F ROS driver

  - `delto3f_gazebo` - Delto Gripper 3F Moveit/Gazebo Simulation

  - `delto_2f_driver` - Delto Gripper 2F ROS driver

  - `delto_2f_gazebo` - Delto Gripper 2F in Gazebo Simulation

 

Some physical measurements (like PID Gain, inertia) may not be accurate. Adjustments may be necessary for a perfect simulation or operation.



## Getting Started


1. **ROS Install** 

- Check this  [ROS 1 Melodic installation site](https://wiki.ros.org/melodic/Installation/Ubuntu).

  
2. **Gazebo Ign Install**

- Check this  [Gazebo installation site](https://classic.gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0).


3. **Moveit Install**
```
sudo apt install ros-melodic-moveit
```

3. **Create a new ROS workspace**:



## How to use Delto Gripper

[![DG3F](https://img.youtube.com/vi/cFbdHVstmg4/0.jpg)](https://www.youtube.com/watch?v=cFbdHVstmg4)

  
## How to Connect Delto Gripper with Robot Arm

 **Connecting Gripper URDF file with robot arm**

make new joint (robot_arm_end_link - gripper_base_link)


**example Delto with robot-arm (Universal Robot)

```xacro

...

</joint>
  <joint name="ee_joint" type="fixed">
    <origin rpy="0 1.57 0" xyz="0 0 0"/>
    <parent link="wrist_3_link"/>
    <child link="delto_base_link"/>
    <!-- <axis xyz="0.0 1.0 0.0"/> -->
 </joint>
 
 ...
  
```
## visualizing the robot arm with delto gripper

```bash
ros2 launch delto_description DG3F.launch
```

```bash
ros2 launch delto_description DG2F.launch
```
