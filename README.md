This package allows to convert non-ROS datasets containing images and other sensor data to a rosbag file. It uses a friendly and simple syntax to which most non-ROS datasets can be conformed, by simple scripting.

To see the available parameters use:

    rosrun dataset2bag dataset2bag --help

# Sensor Types

This package supports the following type of sensors:

* Images (either mono or stereo camera) + calibration
* Odometry
* Imu
* Laser
* Groundtruth

## Images

The converter expects a directory where images should be stored. Images are read in alphabetical order. A video file can also be specified instead of a directory, from which frames will be extracted.

For reading images, the corresponding timestamp of each image is expected. This is supplied in a file containing this data.

Single camera example:

    dataset2bag --images=img --calib=calibration.txt --timestamps=timestamps.txt -o out.bag

### Camera Calibration Syntax

The camera calibration contains the image size, all instrinsic (3x3 matric) and extrinsic (3x3 for rotation and 3x1 for translation) parameters of the camera, plus distortion coefficients (5x1). In the monocular case, the extrinsic parameters should be set to an identity rotation matrix and a zero translation vector.

Syntax:

    WIDTH HEIGHT

    K11 K12 K13
    K21 K22 K23
    K31 K32 K33

    D1 D2 D3 D4 D5

    R11 R12 R13
    R21 R22 R33
    R31 R32 R33

    Tx Ty Tz

### Timestamp File Syntax

This file should contain one line per timestamp with the following format:

    [seconds] [nanoseconds]

## Odometry

Odometry information is used to represent 2D poses of the robot. The data will be stored as a nav_msgs/Odometry message and also published as a TF transform (from /odom to /base_link frames).

Format:

    [seconds] [nanoseconds] [x] [y] [angle]

Units are in metres and radians.

## IMU

This file should contain linear acceleration (m/s^2), angular velocity (rad/s) and orientation (3x3 matrix).

Format:

    [seconds] [nanoseconds] [Ax] [Ay] [Az] [Wx] [Wy] [Wz] [R1] ... [R9]

## Ground-truth

This is used to store poses as geometry_msgs/PoseStamped or geometry_msgs/PoseWithCovarianceStamped. This is again in 2D.

Format without covariance:

    [seconds] [nanoseconds] [x] [y] [theta]

With covariance:

    [seconds] [nanoseconds] [x] [y] [theta] [Cxx] [Cxy] [Cxt] [Cyx] [Cyy] [Cyt] [Ctx] [Cty] [Ctt]

## Laser

This file should first start with some parameters:

    [angle_increment] [ray_count] [min_range] [max_range] [min_angle] [max_angle]

where angles are in degrees and distances in meters. `ray_count` refers to the number of rays contained in one laser scan sample.

Then, for each scan there should be a line as:

    [seconds] [nanoseconds] [range_1] ... [range_N]
    
(where N equals `ray_count`).


