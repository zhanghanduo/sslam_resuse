.. _chapter-overview:

Overview
========

SSLAM package is an optimization-based multi-sensor (mainly stereo camera) state estimator,
which achieves accurate self-localization for autonomous applications.
It supports multiple visual-inertial sensor types (stereo cameras, mono camera + IMU, stereo cameras + IMU).
It contains three modules: camera_models camodocal for camera modeling; SLAM Estimator for real-time localization;
 pose_graph package for re-localization, loop closure and map loading & saving.

The updated version of SSLAM replacing g2o with Ceres (g2o gives occasional crashes) and add optical flow as front end for better tracking, which can be accelerated by GPU. We add IMU and GPS as potential sensor fusion.


.. figure:: ../../image/openstreet.png
   :figwidth: 600px
   :height: 400px
   :align: center

Features
--------

    #. Save & Load functions for familiar scenes with higher accuracy (need GPS initial reference).

    #. multiple sensors support (stereo cameras / mono camera+IMU / stereo cameras+IMU).

    #. visual loop closure.

    #. Option to fuse with GPS and INS data.

Author
------

 `Zhang Handuo <http://zhanghanduo.github.io>`_, from the Robot Vision Group, NTU.

.. figure:: ../../image/demo.gif
   :figwidth: 720px
   :height: 400px
   :align: center
