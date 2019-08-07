.. _ros_examples_create_ikfast_module_from_custom_urdf:

********************************************************************************
Creating an IKfast module from custom urdf
********************************************************************************

.. note::

  Work in progress...

This example is mainly copied from `Choreo's IKFast module generation tutorial <https://github.com/yijiangh/Choreo/blob/7c98fd29120e5ce75d2b8ed17bc49488ad983cb6/framefab_robot/abb/framefab_irb6600/framefab_irb6600_support/doc/ikfast_tutorial.rst>`_. It is intended to show the steps to create an IKfast cpp code bases from an URDF file in a docker environment.

For UR robot, please see .. for its specialized analytical kinematics solver.

1. Generate the ``.urdf`` and Collada file (``.dae``)
============================================================================

.. note::

  Well, getting an extra docker container for ros-kinetic-moveit might not be necessary... I will clean this up later.

On the host machine, let's start all over from the the high-level ``xacro`` file. Let's assume we are working on a Windows machine, and we will use a docker container to access the tools needed for the ``xacro-urdf-collada`` generation pipeline.

.. code-block:: bash

  docker run -it -v "%cd%":/output_urdf moveit/moveit:kinetic-release

(on Unix, change ``"%cd%"`` to ```pwd` ``

This will download the docker image with ``ros-kinetic-moveit`` installed and get you inside the bash commandline. Notice that ``-v "%cd%":/output_urdf`` will mount the current working dir to the docker container as ``/output_urdf``. This will allow sharing files between the host and the container.

.. code-block:: bash

  mkdir catkin_ws && mkdir catkin_ws/src
  cd catkin_ws/src
  git clone https://github.com/ros-industrial/universal_robot.git
  cd .. && catkin_make
  source /catkin_ws/devel/setup.bash

Then, find your ``xacro`` file and run the following to generate the urdf file:

.. code-block:: bash

  roscd ur_description
  cd urdf
  rosrun xacro xacro -i ur5_robot.urdf.xacro -o ur5.urdf

Install collada-to-urdf conversion package:

.. code-block:: bash

  apt-get install ros-kinetic-collada-urdf

Then run the following to generate the ``.dae`` file:

.. code-block:: bash

  rosrun collada_urdf urdf_to_collada ur5.urdf ur5.dae

Then copy the generated ``ur5.urdf`` and ``ur5.dae`` files to the shared dir:

.. code-block:: bash

  mv ur5.urdf /output_urdf && mv ur5.dae /output_urdf

Now we are done with the urdf and dae generation.

2. Use docker to build the new image for ``ros-openrave``
===========================================================

Now, we need another docker container to access the openrave-ikfast tools. First, create a file with the name ``DOCKERFILE`` and paste the following script into the file.

.. code-block::

  FROM personalrobotics/ros-openrave
  RUN apt-get update || true && apt-get install -y --no-install-recommends build-essential python-pip liblapack-dev vim ros-indigo-xacro ros-indigo-collada-urdf && apt-get clean && rm -rf /var/lib/apt/lists/*
  RUN pip install sympy==0.7.1

Open a command line prompt in this dir, and run

.. code-block:: bash

  docker build -t openrave-ros_image .

This will trigger docker to build the image. Here you can remove ``openrave-ros_image`` with any descriptive name.

.. code-block:: bash

  mkdir output
  docker run -it -v "%cd%":/ikfast -v "%cd%"/output:/root/.openrave openrave-ros_image

This line will mount the current working dir (where we store our ``.dae`` and ``.urdf`` files) as ``/ikfast``, as well as mount the ``output`` dir to the ``/root/.openrave`` (this is where ikfast generated files will be saved).

After entering this container, first verify the ``.dae`` file:

.. code-block:: bash

  cd ikfast
  openrave-robot.py ur5.dae --info links

This should give you something like:

.. code-block:: bash

  name           index parents
  -----------------------------------
  world          0
  base_link      1     world
  base           2     base_link
  shoulder_link  3     base_link
  upper_arm_link 4     shoulder_link
  forearm_link   5     upper_arm_link
  wrist_1_link   6     forearm_link
  wrist_2_link   7     wrist_1_link
  wrist_3_link   8     wrist_2_link
  ee_link        9     wrist_3_link
  tool0          10    wrist_3_link
  -----------------------------------
  name           index parents

Now, on the host side, create a xml wrapper for the collada file, named as ``ur5_ik_wrapper.xml``:

.. code-block:: xml

  <robot file="ur5.dae">
          <Manipulator name="ur5">
            <base>base_link</base>
            <effector>tool0</effector>
          </Manipulator>
  </robot>

Finally, let's generate the ikfast cpp files:

.. code-block:: bash

  openrave0.9.py --database inversekinematics --robot=ur5_ik_wrapper.xml --iktype=transform6d --iktests=100

Because we installed ``build-essential`` inside the image, we can ask OpenRAVE to run some IK tests for us. OpenRAVE will automatically compile the plugin after it has generated it and run the tests. Pay attention to the final output. It is something like:

.. code-block:: bash

  openravepy.databases.inversekinematics: testik, success rate: 1.000000, wrong solutions: 0.000000, no solutions: 0.000000, missing solution:
0.670000



Further links
=============
