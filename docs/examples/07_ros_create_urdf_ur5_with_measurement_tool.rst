.. _ros_examples:

********************************************************************************
Creating a URDF with an UR5 robot and a custom end-effector
********************************************************************************

0. Install
==========

Before continuing, make sure you have the following packages installed on your linux system::

  sudo apt-get install ros-kinetic-urdf-tutorial
  sudo apt-get install joint-state-publisher
  sudo apt-get install liburdfdom-tools


1. Export your meshes
=====================

In ROS robot link (and end-effector) geometry is defined with 2 different
meshes: *visual* and *collision*. The visual mesh represents how the robot looks
like and the collision mesh is used for collision detection. You could have the
same collision and visual meshes, but a less detailed collision mesh saves some
processing time while performing collision checks. Also, you might want to built
the collision geometry slightly larger than the visual geometry to guarantee for
safe zones.

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

This will create a ``ur5_with_measurement_tool`` folder which contains a ``package.xml`` and a ``CMakeLists.txt``.
Then open ``package.xml`` and add the following lines after the line ``<buildtool_depend>catkin</buildtool_depend>``.

.. code-block:: xml

  <buildtool_depend>catkin</buildtool_depend>
  <test_depend>roslaunch</test_depend>
  <build_export_depend>joint_state_publisher</build_export_depend>
  <build_export_depend>robot_state_publisher</build_export_depend>
  <build_export_depend>rviz</build_export_depend>
  <build_export_depend>xacro</build_export_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>rviz</exec_depend>
  <exec_depend>xacro</exec_depend>

Optionally, modify ``email`` and ``licence``, ``version`` tags.

Then create 3 folders: ``launch``, ``urdf`` and ``meshes`` (with visual and collision folders)::

  mkdir launch
  mkdir urdf
  mkdir -p meshes/visual
  mkdir -p meshes/collision

Copy your meshes into visual and collision (replace YOURPATH wherever you stored the files)::

  cp /mnt/c/Users/YOURPATH/meshes/visual/measurement_tool.stl meshes/visual/
  cp /mnt/c/Users/YOURPATH/meshes/collision/measurement_tool.stl meshes/collision/


3. Create xacros and generate urdf
==================================

Rather than writing urdf files directly, it is more convinient to write xacro 
files from which urdfs are generated. As its name implies, xacro is a macro 
language. The language allows to use constants, to perform simple math 
operations and to parameterize macros simple by using ``${}``.

For examples please have a look at:
  * http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File

 
First go to the urdf folder and create a xacro file for your tool::

  cd urdf
  pico measurement_tool.xacro

Pico is a terminal based text editor. Paste the following into the file:

.. code-block:: xml

  <?xml version="1.0" encoding="utf-8"?>
  <robot xmlns:xacro="http://ros.org/wiki/xacro">
    <xacro:macro name="measurement_tool" params="prefix flange_name"> <!-- Here we define the 2 parameters of the macro -->
      <joint name="${prefix}measurement_tool_joint" type="fixed"> <!-- Create a fixed joint with a parameterized name. -->
        <parent link="${flange_name}"/> <!-- The parent link must be read from the robot model it is attached to. -->
        <child link="${prefix}measurement_tool"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>  <!-- The tool is directly attached to the flange. -->
      </joint>
      <link name="${prefix}measurement_tool">
        <visual>
          <geometry>
            <!-- The path to the visual meshes in the package. -->
            <mesh filename="package://ur5_with_measurement_tool/meshes/visual/measurement_tool.stl"/> 
          </geometry>
        </visual>
        <collision>
          <geometry>
            <!-- The path to the collision meshes in the package. -->
            <mesh filename="package://ur5_with_measurement_tool/meshes/collision/measurement_tool.stl"/>
          </geometry>
        </collision>
      </link>
    </xacro:macro>
  </robot>

Explanation:

The end-effector only consists of one fixed joint and the link geometry. We create a
parameterized macro with 2 parameters (prefix, flange_name) because maybe once 
we want to attach the tool to a different robot with a different flange name or,
if we want to use the end-effector twice in the same urdf we would need to set a
prefix to differenciate. Whatever is defined like ``${}`` will later be replaced
when generating the urdf.

Now we create a new xaxro file, which combines the ur5 with the end-effector::

  pico ur5_with_measurement_tool.xacro

Paste the following:

.. code-block:: xml

  <?xml version="1.0"?>
  <robot xmlns:xacro="http://ros.org/wiki/xacro" name="ur5_with_measurement_tool">

    <!-- ur5 -->
    <xacro:include filename="$(find ur_description)/urdf/ur5.urdf.xacro" />
    <!-- end-effector -->
    <xacro:include filename="measurement_tool.xacro" />

    <!-- ur5 -->
    <xacro:ur5_robot prefix="" joint_limited="true"/>
    <!-- end-effector -->
    <!-- Here we include the end-effector by setting the parameters -->
    <!-- TODO: check end-effector link name of robot -->
    <xacro:measurement_tool prefix="" flange_name="tool0"/>
    
    <!-- define the ur5's position and orientation in the world coordinate system -->
    <link name="world" />
    <joint name="world_joint" type="fixed">
      <parent link="world" />
      <child link = "base_link" /> <!-- TODO: check base_link name of robot -->
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
    </joint>
  </robot>


Now create the urdf.::

  rosrun xacro xacro --inorder -o ur5_with_measurement_tool.urdf ur5_with_measurement_tool.xacro

This will create ur5_with_measurement_tool.urdf in the directory.


4. View urdf
============

Create display.launch in directory::

  cd ..
  mkdir launch
  cd launch
  pico display.launch

paste the following:

.. code-block:: xml

  <launch>

    <arg name="model" default="$(find ur5_with_measurement_tool)/urdf/ur5_with_measurement_tool.urdf"/>
    <arg name="gui" default="true" />
    <arg name="rvizconfig" default="$(find urdf_tutorial)/rviz/urdf.rviz" />

    <param name="robot_description" command="$(find xacro)/xacro --inorder $(arg model)" />
    <param name="use_gui" value="$(arg gui)"/>

    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" required="true" />

  </launch>

Now we need to source the package path in our catkin workspace::

  cd ~/robotic_setups
  catkin_make
  source devel/setup.bash

And then run::

  roslaunch ur5_with_measurement_tool display.launch


.. figure:: 07_urdf_tool_01.jpg
    :figclass: figure
    :class: figure-img img-fluid

    Screenshot of RViz showing the ur5 with the custom end-effector.


Further links
=============

* http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch
* http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
* http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file



