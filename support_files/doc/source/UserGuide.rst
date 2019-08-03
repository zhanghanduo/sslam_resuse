.. _chapter-userguide:

USER GUIDE
==========

.. contents:: Contents
   :local:
   :backlinks: none


0. Source Code Structure
------------------------
::

    ├── README.md
    ├── package.xml
    ├── CMakeLists.txt
    ├── cmake
    │   ├── FindEigen.cmake
    │   └── FindSphinx.cmake
    ├── Config                  Find
    │   ├── bus
    │   ├── bus2
    │   ├── euroc
    │   ├── honda
    │   ├── kitti_odom
    │   ├── kitti_raw
    │   └── zed
    ├── launch
    │   ├── bus_core.launch
    │   ├── bus_global_core.launch
    │   ├── bus_global.launch
    │   └── bus.launch
    ├── docs
    │   ├── html
    │   ├── latex
    │   └── xml
    ├── camera_models
    │   ├── include
    │   ├── readme.md
    │   └── src
    ├── slam_estimator
    │   ├── cmake-build-debug
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src
    ├── pose_graph
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src
    ├── global_fusion
    │   ├── CMakeLists.txt
    │   ├── models
    │   ├── package.xml
    │   ├── src
    │   └── ThirdParty
    └── support_files
        ├── brief_k10L6.bin
        ├── brief_pattern.yml
        ├── doc
        └── image

1. Config File
--------------

Configuration file lies in ``config/`` directory, stored in ``yaml`` format.



- ``imu``: set to ``0`` means ignores **IMU input**, ``1`` means uses **IMU input**.

- ``ins``: set to ``0`` means ignores **INSPVA input**, ``1`` means uses **INSPVA input**.

- ``num_of_cam``: set to ``1`` means uses **monocular camera**, ``2`` means uses **stereo camera**.

- ``cubicle``: set to ``0`` means ignores **object detection** input, ``1`` means uses **object detection** input.

    .. NOTE ::

        ``cubicle`` requires the installation and operation of both `obstacle msgs <https://gitlab.com/ugv_stereo/obstacle_msgs.git>`_ package
         and `cubicle detect <https://gitlab.com/ugv_stereo/cubicle_detect.git>`_ package.

- ``gps_initial``: set to 1, then the initial robot body pose is aligned with **GPS pose** at the initial moment, for visualization and evaluation;
otherwise SLAM starts at local frame.

- ``imu_topic``: set IMU topic name if use IMU data. IMU message is: ``sensor_msgs/Imu``.

- ``ins_topic``: set INS topic name if use INS data. INSPVA message is: ``msg_novatel_inspva``.

- ``image0_topic``: set left image topic name.

- ``image1_topic``: set right image topic name, if use stereo camera.

- ``cubicle_topic``: set object detection topic name, if enable  ``cubicle`` flag. Cubicle message is: ``obstacle_msgs/MapInfo``.

- ``gps_topic``: set GPS pose topic name, if enable ``gps_initial`` flag. GPS message is ``geometry_msgs/PoseWithCovarianceStamped``.



2. Bus Demo
-----------

.. code-block:: bash

    roslaunch sslam_estimator bus.launch


3. EuRoC Example
----------------

.. _section-euroc:

Download `EuRoC MAV Dataset <http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets>`_ to ``YOUR_DATASET_FOLDER``. Take ``MH_01`` for example, you can run SSLAM with three sensor types (monocular camera + IMU, stereo cameras + IMU and stereo cameras).
Open 3 terminals, run sslam_estimator, rviz and play the bag file respectively.
Green path is VIO odometry; yellow path is odometry under visual loop closure.


3.1 Monocualr camera + IMU

.. code-block:: bash

    roslaunch sslam_estimator bus_imu.launch
    rosbag play YOUR_DATASET_FOLDER/bus_03_with_imu.bag


3.2 Stereo cameras + IMU

.. code-block:: bash

    roslaunch sslam_estimator bus_imu.launch
    rosbag play YOUR_DATASET_FOLDER/bus_03_with_imu.bag

3.3 Stereo cameras

.. code-block:: bash

    roslaunch sslam_estimator bus.launch
    rosbag play YOUR_DATASET_FOLDER/bus_01.bag


4. KITTI Example
----------------

.. _section-kitti:

4.1 KITTI Odometry (Stereo)
Download `KITTI Odometry dataset <http://www.cvlibs.net/datasets/kitti/eval_odometry.php>`_ to YOUR_DATASET_FOLDER. Take sequences 00 for example,
Open two terminals, run sslam_estimator and rviz respectively.

.. code-block:: bash

    rosrun sslam_estimator kitti_odom_test ~/catkin_ws/src/sslam_resuse/slam_estimator/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/

4.2 KITTI GPS Fusion (Stereo + GPS)
Download `KITTI raw dataset <http://www.cvlibs.net/datasets/kitti/raw_data.php>`_ to ``YOUR_DATASET_FOLDER``.
Open three terminals, run sslam_estimator, global fusion and rviz respectively.
Green path is VIO odometry; blue path is odometry under GPS global fusion.

.. code-block:: bash

    roslaunch sslam_estimator bus_global.launch


5. SSLAM on car demonstration with GPS
--------------------------------------
.. _section-sslam:

Run SSLAM odometry, rviz and play the bag file respectively.
Green path is VIO odometry; red path is odometry under visual loop closure.

.. code-block:: bash

    roslaunch sslam_estimator car.launch
    rosbag play YOUR_DATASET_FOLDER/car.bag


