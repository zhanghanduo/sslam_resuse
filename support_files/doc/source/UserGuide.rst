USER GUIDE
==========






1. EuRoC Example
----------------

Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) to YOUR_DATASET_FOLDER. Take MH_01 for example, you can run VINS-Fusion with three sensor types (monocular camera + IMU, stereo cameras + IMU and stereo cameras).
Open four terminals, run sslam_estimator odometry, visual loop closure(optional), rviz and play the bag file respectively.
Green path is VIO odometry; yellow path is odometry under visual loop closure.

1.1 Monocualr camera + IMU

.. code-block:: bash
    roslaunch sslam_estimator bus_imu.launch
    rosbag play YOUR_DATASET_FOLDER/bus_03_with_imu.bag


1.2 Stereo cameras + IMU

.. code-block:: bash
    roslaunch sslam_estimator bus_imu.launch
    rosbag play YOUR_DATASET_FOLDER/bus_03_with_imu.bag

1.3 Stereo cameras

.. code-block:: bash
    roslaunch sslam_estimator bus.launch
    rosbag play YOUR_DATASET_FOLDER/bus_01.bag


2. KITTI Example
----------------

2.1 KITTI Odometry (Stereo)
Download `KITTI Odometry dataset <http://www.cvlibs.net/datasets/kitti/eval_odometry.php>`_ to YOUR_DATASET_FOLDER. Take sequences 00 for example,
Open two terminals, run sslam_estimator and rviz respectively.

.. code-block:: bash
    rosrun sslam_estimator kitti_odom_test ~/catkin_ws/src/sslam_resuse/slam_estimator/config/kitti_odom/kitti_config00-02.yaml YOUR_DATASET_FOLDER/sequences/00/

2.2 KITTI GPS Fusion (Stereo + GPS)
Download `KITTI raw dataset
         <http://www.cvlibs.net/datasets/kitti/raw_data.php>`_ to YOUR_DATASET_FOLDER.
Open three terminals, run sslam_estimator, global fusion and rviz respectively.
Green path is VIO odometry; blue path is odometry under GPS global fusion.

.. code-block:: bash
    roslaunch sslam_estimator bus_global.launch


3. SSLAM2 on car demonstration
------------------------------
Run sslam odometry, visual loop closure(optional), rviz and play the bag file respectively.
Green path is VIO odometry; red path is odometry under visual loop closure.
.. code-block:: bash
    roslaunch sslam_estimator car.launch
    rosbag play YOUR_DATASET_FOLDER/car.bag


