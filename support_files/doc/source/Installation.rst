.. _chapter-installation:

============
Installation
============

Getting the source code
=======================
.. _section-source:

If you want the latest version, you can clone the git repository

.. code-block:: bash

       git clone https://gitlab.com/ugv_stereo/sslam_resuse.git -b devel

.. _section-dependencies:

Dependencies
============

  .. NOTE ::

    SSLAM package requires ROS environment and a **fully C++11-compliant**
    compiler. Our code has been fully tested under Ubuntu 16.04, ROS Kinetic.

1. Ceres Solver
---------------

Follow `Ceres Installation
<http://ceres-solver.org/installation.html>`_.

2. OpenCV 3.4
---------

Follow `Opencv Installation
<https://zhanghanduo.github.io/post/new_system>`_.

3. Cereal Serialization
-----------------------

It is used to serialize / deserialize map points swiftly and lightly.
Download it from `github repository <https://github.com/USCiLab/cereal.git>`_.

  .. NOTE ::
    Under Ubuntu 18.04, you have to edit CMakeLists.txt and change line 4 from ``OFF`` to ``ON`` to avoid compiling error.

.. code-block:: bash

        mkdir build && cd build
        cmake ..
        make -j8
        sudo make install

4. Common_utils package
------------------------

Download it from `gitlab/ugv_stereo/obstacle_msg <https://gitlab.com/ugv_stereo/common_utils.git>`_ into catkin workspace.

Read the documentation for dependencies and configuration for correct camera calibration file and ROS topic interface.

5. Obstacle_msg package
------------------------

Download it from `gitlab/ugv_stereo/obstacle_msg <https://gitlab.com/ugv_stereo/obstacle_msgs.git>`_ into catkin workspace.

6. Cubicle_detect package
-------------------------

Download it from `gitlab/ugv_stereo/cubicle_detect <https://gitlab.com/ugv_stereo/cubicle_detect.git>`_ into catkin workspace.

7. Build SSLAM package
----------------------
- Catkin tools (suggested approach)

    + Initialize work space
        .. code-block:: bash

            sudo apt install python-wstool python-catkin-tools ros-kinetic-cmake-modules autoconf
            mkdir -p ~/catkin_ws/src
            cd ~/catkin_ws
            catkin init
            catkin config --extend /opt/ros/kinetic
            catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
            catkin config --merge-devel

    + Compile
        .. code-block:: bash

            catkin build


- ROS native catkin package toolchain

    + Initialize work space
        .. code-block:: bash

            mkdir -p ~/catkin_ws/src
            cd ~/catkin_ws

    + Compile
        .. code-block:: bash

            catkin_make

