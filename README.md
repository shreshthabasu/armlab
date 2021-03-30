# Armlab

The objective of this project is to program a 6 DOF robot manipulator to perform pick-and-place of colored cubes based on the Kinect sensor. The tasks required to be complete this project can be broken down into two categories:

1. **The Kinect Sensor**: consists of an RGB camera along with an IR laser projector and IR camera. The opensource `freenect` driver is used to connect to the camera and capture images.

    * **Calibration**:
        * Affine transformation was calculated between the depth image and the RGB image to line them up
        * Intrinsic Calibration was performed using OpenCV libraries to find the transformation between the camera coordinates and the pixel coordinates on the image
        * Extrinsic Calibration: April Tags along with the OpenCV libraries were used to achieve automatic extrinsic calibration to find the transformation between the camera coordinates and the arm coordinates.

    * **Block and Color Detection**:
        * Block Detection: Use OpenCV for contour detection to detect the blocks. Filter the depth values of the detected blocks to find out how many blocks are stacked on top of each other.
        * Color Detection: Use HSV values and filter the ranges for different colors.

2. **Robot Manipulator**: Generate a trajectory of joint positions for the manipulator to sequence through to reach the desired goal pose

    * **Trajectory Planning**: Use a cubic spline to implement trajectory smoothening to limit jerky motions between different joint positions. The cubic spline creates smooth position and velocity profiles.

    * **Forward/Inverse Kinematics**: Update DH parameters to achieve the orientation and position of end effector. For IK, the joint positions for end effector pose is calculated
