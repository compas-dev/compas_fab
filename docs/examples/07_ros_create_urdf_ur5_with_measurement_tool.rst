.. _ros_examples:

********************************************************************************
Creating a URDF with an UR5 robot and a custom end-effector
********************************************************************************

0. Install
==========

Before continuing, make sure you have the following packages installed on your linux system.

  sudo apt-get install ros-kinetic-urdf-tutorial
  sudo apt-get install joint-state-publisher
  sudo apt-get install liburdfdom-tools


1. Export your meshes
=====================

In ROS robot link (and end-effector) geometry is defined with 2 different meshes: *visual* and *collision*. The visual mesh represents how the robot looks like and the collision mesh is used for collision detection. You could have the same collision and visual meshes, but a less detailed collision mesh saves some processing time in the collision checks. Also, you might want to built the collision geometry slightly larger than the visual geometry to guarantee for safe zones.

Before exporting, please position your end-effector, such that the connection point to the flange (tool0) is in (0,0,0). The geometry of your end-effector has to be defined in *meters*. Then export both visual and a collision meshes of your end-effector in a ROS-friendly format, like .stl or .obj (see below).

.. figure:: 07_urdf_tool_00.jpg
    :figclass: figure
    :class: figure-img img-fluid

    Screenshot of exporting a tool geometry from Rhino3D positioned in (0,0,0).


2. Prepare your catkin workspace
================================

Open your linux bash console and go to your home directory::

  cd ~

If not yet there, make a new catkin workspace for all your robotic setups::

  mkdir -p robotic_setups/src
  cd robotic_setups
  catkin_make

Then go to your src folder and make a package with your new setup ``ur5_with_measurement_tool``::

  cd src
  catkin_create_pkg ur5_with_measurement_tool

This will create a ``ur5_with_measurement_tool`` folder which contains a ``package.xml`` and a ``CMakeLists.txt``, which have been partially filled out with the information you gave ``catkin_create_pkg``.
Then open ``package.xml`` and add the following lines after the line ``<buildtool_depend>catkin</buildtool_depend>``.

.. code-block:: xml
  <package>


  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roslaunch</build_depend>
  <run_depend>joint_state_publisher</run_depend>
  <run_depend>robot_state_publisher</run_depend>
  <run_depend>rviz</run_depend>
  <run_depend>xacro</run_depend>

Optionally, modify ``email`` and ``licence``, ``version`` tags.

Then create 3 folders: ``launch``, ``urdf`` and ``meshes`` (with visual and collision folders). LAUNCH NECESSARY??::

  mkdir launch
  mkdir urdf
  mkdir -p meshes/visual
  mkdir -p meshes/collision

Copy your meshes into visual and collision::

  cp /mnt/c/Users/YOURPATH/meshes/visual/measurement_tool.stl meshes/visual/
  cp /mnt/c/Users/YOURPATH/meshes/collision/measurement_tool.stl meshes/collision/


3. Create xacros and generate urdf
==================================

What are xacros

Go to the urdf folder and create the xacro for your tool::

  cd urdf
  subl measurement_tool.xacro

This will open sublime text editor. Paste the following into the file:

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <robot xmlns:xacro="http://ros.org/wiki/xacro" name="measurement_tool">
    <xacro:macro name="measurement_tool" params="prefix flange_name">
      <joint name="${prefix}measurement_tool_joint" type="fixed">
        <parent link="${flange_name}"/>
        <child link="${prefix}measurement_tool"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>  
      </joint>
      <link name="${prefix}measurement_tool">
        <visual>
          <geometry>
            <mesh filename="package://ur5_with_measurement_tool/meshes/visual/measurement_tool.stl"/>
          </geometry>
        </visual>
        <collision>
          <geometry>
            <mesh filename="package://ur5_with_measurement_tool/meshes/collision/measurement_tool.stl"/>
          </geometry>
        </collision>
      </link>
    </xacro:macro>
  </robot>

This are a fixed joint with the link including the geometry. Variables will a "$" sign can be set via arguments.
Now create a new xaxro file

  subl ur5_with_measurement_tool.xacro

.. code-block:: xml

  <?xml version="1.0"?>
  <robot xmlns:xacro="http://ros.org/wiki/xacro" name="ur5_with_measurement_tool" params="prefix flange_name">

    <!-- ur5 -->
    <xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro" />
    <!-- end-effector -->
    <xacro:include filename="measurement_tool.xacro" />

    <!-- ur5 -->
    <xacro:ur5_robot prefix="" joint_limited="true"/>
    <!-- end-effector -->
    <xacro:measurement_tool prefix="" flange_name="flange"/>
    
    <!-- define the ur5's position and orientation in the world coordinate system -->
    <link name="world" />
    <joint name="world_joint" type="fixed">
      <parent link="world" />
      <child link = "base_link" /> 
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
    </joint>
    
  </robot>

Now we need to source the package as path::

  cd ~/robotic_setups
  catkin_make
  source devel/setup.bash

Go back in the urdf folder::

  cd src/ur5_with_measurement_tool/urdf

Now create the urdf.:

  rosrun xacro xacro --inorder -o ur5_with_measurement_tool.urdf ur5_with_measurement_tool.xacro




get display.launch

cd launch
wget https://raw.githubusercontent.com/ros/urdf_tutorial/master/launch/display.launch

http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch
roslaunch urdf_tutorial display.launch model:=urdf/01-myfirst.urdf



 So, in order to attach an end-effector to the robot model, you have to export a visual and a collision mesh of your end-effector.



sudo apt-get install liburdfdom-tools

Further links
* http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
* http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file


