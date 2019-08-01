part of [camodocal](https://github.com/hengli/camodocal)

[Google Ceres](http://ceres-solver.org) is needed.

# Undistortion:

See [Camera.h](https://github.com/dvorak0/camera_model/blob/master/include/camodocal/camera_models/Camera.h) for general interface:

 - liftProjective: Lift points from the image plane to the projective space.
 - spaceToPlane: Projects 3D points to the image plane (Pi function)

