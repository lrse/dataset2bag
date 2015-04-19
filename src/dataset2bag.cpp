#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <image_geometry/pinhole_camera_model.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/program_options.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

namespace po = boost::program_options;

bool process_args(int argc, char** argv, po::variables_map& options)
{
  po::options_description options_description("Options");
  options_description.add_options()
    ("images,l", po::value<std::string>()->required(), "pattern of left images or path to video (or mono-camera)")

    ("images_right,r", po::value<std::string>(), "pattern of right images or path to video")

    ("odometry", po::value<std::string>(), "path to file containing 2D odometry data")

    ("output,o", po::value<std::string>()->required(), "output bag file")

    ("calib,c", po::value<std::string>()->required(), "left camera calibration parameters file")

    ("calib_right", po::value<std::string>(), "right camera calibration parameters file")

    ("timestamps,t", po::value<std::string>(), "path to file containing timestamps")
    ("framerate,f", po::value<float>(), "frame-rate to assume when not supplying timestamps from file")

    ("imu", po::value<std::string>(), "path to IMU data file")
    ("groundtruth", po::value<std::string>(), "path to ground-truth data file")
    ("gt-with-covariance", "if specified, the ground-truth file contains covariance information")

    //("static-transforms", po::value<std::string>(), "path to file containing static transforms between sensors")
    //("laser", po::value<std::string>(), "path to laser scans file")

    ("help,h", "show this help")
  ;

  try {
    po::store(po::parse_command_line(argc, argv, options_description), options);
    po::notify(options);
  }
  catch(po::error& e) { 
    std::cerr << options_description << std::endl << std::endl;
    std::cerr << std::endl << "ERROR: " << e.what() << std::endl;
    return false;
  }

  if ( options.count("help") ) {
    std::cerr << options_description << std::endl;
    return false;
  }

  return true;
}

/**
 * @brief expects a file with a list of timestamps (<seconds> <nanoseconds>).
 */
std::vector<ros::Time> loadTimestamps(const std::string& filename)
{
  std::vector<ros::Time> timestamps;

  std::ifstream ifs(filename.c_str());

  while (!ifs.eof()) {
    uint32_t sec, nsec;
    ifs >> sec >> nsec;
    //std::cout << sec << "." << nsec << std::endl;
    timestamps.push_back(ros::Time(sec, nsec));
  }

  return timestamps;
}

/**
 * @brief load a timestamp list for a number of frames
 * in regular intervals.
 * 
 * @param nFrames
 *   number of frames
 * 
 * @param framerate
 *   framerate in frames / second
 */
std::vector<ros::Time> loadTimestamps( size_t nFrames, double framerate )
{
  std::vector<ros::Time> timestamps;

  // set initial time (in seconds)
  // 0 is not allowed...
  //~ std::cout << "min time: " << ros::TIME_MIN << std::endl;
  double timestamp = ros::TIME_MIN.toSec();

  for(size_t k=0; k<nFrames; k++) {
    timestamps.push_back( ros::Time( timestamp ) );
    timestamp += framerate;
  }

  return timestamps;
}

sensor_msgs::CameraInfo loadCameraCalibration( const std::string& filename, size_t width, size_t height )
{
  boost::array<double, 9> intrinsics;
  std::vector<double> dist_coefficients(5);
  boost::array<double, 9> rectification;

  std::ifstream ifs( filename.c_str() );
  for(int i = 0; i < 9; ++i) ifs >> intrinsics[i];
  for(int i = 0; i < 5; ++i) ifs >> dist_coefficients[i];
  for(int i = 0; i < 9; ++i) ifs >> rectification[i];

  // create CameraInfo message
  sensor_msgs::CameraInfo camera_info;

  camera_info.height = height;
  camera_info.width = width;
  camera_info.distortion_model = "plumb_bob";
  camera_info.D = dist_coefficients;
  camera_info.K = intrinsics;
  camera_info.R = rectification;

  /* build P from K */
  cv::Mat K(3, 3, CV_64FC1, intrinsics.c_array());
  cv::Mat P = K * cv::Mat::eye(3, 4, CV_64FC1);
  for (int i = 0; i < P.total(); i++) camera_info.P[i] = *(P.ptr<double>(0) + i);

  return camera_info;
}

void saveStream( cv::VideoCapture& capture, sensor_msgs::CameraInfo camera_info, const std::vector<ros::Time>& times, const std::string& frame_id, const std::string& topic, rosbag::Bag& bag )
{
  cv_bridge::CvImage ros_image;

  uint seq = 0;

  while (capture.read(ros_image.image) /*&& seq < 500*/)
  {
    std::cout << "loading image " << seq << "/" << times.size() << std::endl;
    cv::cvtColor(ros_image.image, ros_image.image, CV_BGR2RGB);
    ros_image.encoding = "rgb8";

    sensor_msgs::ImagePtr ros_image_msg;

    // create image message

    ros_image_msg = ros_image.toImageMsg();
    ros_image_msg->header.seq = seq;
    ros_image_msg->header.stamp = times[ seq ];
    ros_image_msg->header.frame_id = frame_id;

    bag.write(topic + "/image_raw", times[ seq ], ros_image_msg);

    // create CameraInfo message

    //~ sensor_msgs::CameraInfo camera_info;
    camera_info.header.seq = seq;
    camera_info.header.stamp = times[ seq ];
    camera_info.header.frame_id = frame_id;

    bag.write(topic + "/camera_info", times[ seq ], camera_info);

    seq++;
  }
}

void saveOdometry(const std::string& filename, rosbag::Bag& bag)
{
  std::ifstream ifs(filename);

  size_t seq = 0;
  while(!ifs.eof()) {
    float x, y, yaw;
    uint32_t sec, nsec;

    ifs >> sec >> nsec >> x >> y >> yaw;

    //std::cout << sec << "." << nsec << " " << x << " " << y << " " << yaw << std::endl;

    ros::Time stamp(sec, nsec);

    // write odometry message
    nav_msgs::Odometry odo_msg;
    odo_msg.header.stamp = stamp;
    odo_msg.header.frame_id = "odom";
    odo_msg.child_frame_id = "base_link";
    odo_msg.pose.pose.position.x = x;
    odo_msg.pose.pose.position.y = y;
    odo_msg.pose.pose.position.z = 0;
    odo_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    bag.write("/robot/odometry", stamp, odo_msg);

    // write tf message
    geometry_msgs::TransformStamped tmsg;
    tmsg.header.stamp = stamp;
    tmsg.header.frame_id = "odom";
    tmsg.child_frame_id = "base_link";
    tmsg.transform.translation.x = odo_msg.pose.pose.position.x;
    tmsg.transform.translation.y = odo_msg.pose.pose.position.y;
    tmsg.transform.translation.z = 0;
    tmsg.transform.rotation = odo_msg.pose.pose.orientation;

    tf::tfMessage tf_msg;
    tf_msg.transforms.push_back(tmsg);
    tf_msg.transforms.back().header.frame_id = tf::resolve("", tf_msg.transforms.back().header.frame_id);
    tf_msg.transforms.back().child_frame_id = tf::resolve("", tf_msg.transforms.back().child_frame_id);

    bag.write("/tf", stamp, tf_msg);

    seq++;
  }
}

void saveIMU(const std::string& filename, rosbag::Bag& bag)
{
  std::ifstream ifs(filename);

  size_t seq = 0;
  while(!ifs.eof()) {
    float acc[3], gyro[3], R[9];
    uint32_t sec, nsec;

    ifs >> sec >> nsec;
    ros::Time stamp(sec, nsec);

    for (int i = 0; i < 3; i++) ifs >> gyro[i];
    for (int i = 0; i < 3; i++) ifs >> acc[i];
    for (int i = 0; i < 9; i++) ifs >> R[i];

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = stamp;
    imu_msg.header.seq = seq;
    imu_msg.header.frame_id = "imu";
    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];
    imu_msg.linear_acceleration.x = acc[0];
    imu_msg.linear_acceleration.y = acc[1];
    imu_msg.linear_acceleration.z = acc[2];

    tf::Quaternion Q;
    tf::Matrix3x3 Rmat(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
    Rmat.getRotation(Q);
    tf::quaternionTFToMsg(Q, imu_msg.orientation);

    bag.write("/imu", stamp, imu_msg);
    seq++;
  }
}

void saveGT(const std::string& filename, rosbag::Bag& bag, bool with_covariance)
{
  std::ifstream ifs(filename);

  if (with_covariance) std::cout << "... with covariance" << std::endl;

  size_t seq = 0;
  while(!ifs.eof()) {
    uint32_t sec, nsec;
    float x, y, theta;
    float cov[9];

    ifs >> sec >> nsec;
    //std::cout << sec << "." << nsec << std::endl;
    ros::Time stamp(sec, nsec);

    ifs >> x >> y >> theta;
    if (with_covariance) {
      for (int i = 0; i < 9; i++) ifs >> cov[i];

      geometry_msgs::PoseWithCovarianceStamped pose;
      pose.header.stamp = stamp;
      pose.header.seq = seq;
      pose.header.frame_id = "groundtruth";

      pose.pose.pose.position.x = x;
      pose.pose.pose.position.y = y;
      pose.pose.pose.position.z = 0;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(theta).normalized(), pose.pose.pose.orientation);

      for (int i = 0; i < 36; i++) pose.pose.covariance[i] = 999999.0; // initialize unknown covariance to big values

      pose.pose.covariance[0] = cov[0]; // Cxx
      pose.pose.covariance[1] = cov[1]; // Cxy
      pose.pose.covariance[5] = cov[2]; // Cxt (t = rot around Z)

      pose.pose.covariance[6] = cov[3]; // Cyx
      pose.pose.covariance[7] = cov[4]; // Cyy
      pose.pose.covariance[11] = cov[5]; // Cyt

      pose.pose.covariance[30] = cov[6]; // Ctx
      pose.pose.covariance[31] = cov[7]; // Cty
      pose.pose.covariance[35] = cov[8]; // Ctt

      bag.write("/groundtruth_pose", stamp, pose);
    }
    else {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = stamp;
      pose.header.seq = seq;
      pose.header.frame_id = "groundtruth";

      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0;
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(theta), pose.pose.orientation);

      bag.write("/groundtruth_pose", stamp, pose);
    }
  }

  seq++;
}

/* not yet finished */
#if 0
void saveTransforms(const std::string& filename, rosbag::Bag& bag)
{
  std::ifstream ifs(filename);

  size_t seq = 0;
  while(!ifs.eof()) {
    uint32_t sec, nsec;
    ifs >> sec >> nsec;
    //std::cout << sec << "." << nsec << std::endl;
    ros::Time stamp(sec, nsec);
  }
}
#endif

/* not yet finished */
#if 0
void saveLaser(const std::string& filename, rosbag::Bag& bag, bool with_covariance)
{
  std::ifstream ifs(filename);

  size_t seq = 0;
  while(!ifs.eof()) {
    uint32_t sec, nsec;
    ifs >> sec >> nsec;
    ros::Time stamp(sec, nsec);

    float delta_angle;
    size_t scan_count;
    ifs >> delta_angle >> scan_count;

    sensor_msgs::LaserScan laser_msg;
    laser_msg.header.stamp = stamp;
    laser_msg.header.seq = seq;
    laser_msg.angle_increment = delta_angle;

    laser_msg.ranges.resize(scan_count);
    for (int i = 0; i < scan_count; i++) ifs >> laser_msgs.ranges[i];

    laser_msg.angle_min =

    seq++;
  }
}
#endif

int main(int argc, char** argv)
{
  po::variables_map options;

  if ( not process_args(argc, argv, options) )
    return EXIT_FAILURE;

  // open video captures

  std::string left_images = options["images"].as<std::string>();
  cv::VideoCapture capture_left( left_images );

  size_t width = (size_t) capture_left.get( CV_CAP_PROP_FRAME_WIDTH );
  size_t height = (size_t) capture_left.get( CV_CAP_PROP_FRAME_HEIGHT );
  size_t nFrames = (size_t) capture_left.get( CV_CAP_PROP_FRAME_COUNT );

  std::cout << "-- Loading " << nFrames << " frames of size " << width << "x" << height << std::endl;

  std::string left_calib = options["calib"].as<std::string>();
  sensor_msgs::CameraInfo cam_info_left = loadCameraCalibration( left_calib, width, height );

  std::cout << "Parsing timestamps ..." << std::endl;

  std::vector<ros::Time> timestamps = options.count("timestamps") ? loadTimestamps( options["timestamps"].as<std::string>() ) : loadTimestamps( nFrames, 1 / options["framerate"].as<float>() );

  std::cout << "ts: " << timestamps.size() << std::endl;
  assert( nFrames == timestamps.size() );

  // open a new bag file in write mode
  rosbag::Bag bag;

  bag.open(options["output"].as<std::string>(), rosbag::bagmode::Write);

  if (options.count("odometry")) {
    std::cout << "Parsing odometry file..." << std::endl;
    saveOdometry(options["odometry"].as<std::string>(), bag);
  }

  if (options.count("imu")) {
    std::cout << "Parsing IMU data..." << std::endl;
    saveIMU(options["imu"].as<std::string>(), bag);
  }

  if (options.count("groundtruth")) {
    std::cout << "Parsing ground-truth data..." << std::endl;
    saveGT(options["groundtruth"].as<std::string>(), bag, options.count("gt-with-covariance"));
  }

#if 0
  if (options.count("static-transforms")) {
    std::cout << "Parsing static transforms file..." << std::endl;
    saveTransforms(options["static-transforms"].as<std::string>(), bag);
  }
#endif

#if 0
  if (options.count("laser")) {
    std::cout << "Parsing laser data..." << std::endl;
    saveLaser(options["laser"].as<std::string>(), bag);
  }
#endif

  if ( options.count("images_right") )
  {
    assert( options.count("calib_right") );

    std::cout << "Parsing stereo images ..." << std::endl;

    std::string right_images = options["images_right"].as<std::string>();
    cv::VideoCapture capture_right( right_images );

    std::string right_calib = options["calib_right"].as<std::string>();
    sensor_msgs::CameraInfo cam_info_right = loadCameraCalibration( right_calib, width, height );

    saveStream( capture_left, cam_info_left, timestamps, "camera_left", "/stereo/left", bag );
    saveStream( capture_right, cam_info_right, timestamps, "camera_right", "/stereo/right", bag );
  }
  else
  {
    std::cout << "Parsing mono images ..." << std::endl;

    saveStream( capture_left, cam_info_left, timestamps, "camera", "/camera", bag );
  }

  bag.close();
}
