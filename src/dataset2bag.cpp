#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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

    // (Example: the pattern could be form image_%3d.png for images of the form image_000.png)
    ("images,l", po::value<std::string>()->required(), "pattern of left images or path to video (or mono-camera)")

    // (Example: the pattern could be form image_%3d.png for images of the form image_000.png)
    ("images_right,r", po::value<std::string>(), "pattern of right images or path to video")

    ("poses,p", po::value<std::string>(), "2D odometry/ground truth in CSV format (x,y,theta)")

    ("output,o", po::value<std::string>()->required(), "output bag file")

    //~ ("skip,s", po::value<int>()->default_value(0), "skip an ammount of frames for each frame processed")

    ("calib,c", po::value<std::string>()->required(), "left camera calibration parameters file")

    ("calib_right,c", po::value<std::string>(), "right camera calibration parameters file")

    /*("image-timestamps", po::value<std::string>(), "file with one timestamp per frame")
    ("image-delta", po::value<int>(), "artificial delta between frames in milliseconds")
    
    ("poses-have-timestamps", "indicates that poses file have a leading field including timestamp")
    ("pose-delta", po::value<int>(), "artificial delta between poses in milliseconds")*/
    
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
 * @brief expects a file with a list of whitespace separated doubles
 * representing timestamps.
 */
std::vector<ros::Time> loadTimestamps( const std::string& filename )
{
  std::vector<ros::Time> timestamps;

  std::ifstream ifs( filename.c_str() );

  while ( ifs ) {
    double timestamp;
    ifs >> timestamp;
    timestamps.push_back( ros::Time( timestamp ) );
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

/**
 * @brief intrinsic calibration file is expected in row major order:
 * 
 *   K11 K12 K13 K21 K22 K23 K31 K32 K33
 *
 * @param width
 *   image width in pixels.
 *
 * @param height
 *   image height in pixels.
 * 
 * @param baseline
 *   camera baseline. 0 for mono camera and for left stereo camera,
 *   used to specify the relative position of the right stereo camera.
 */
sensor_msgs::CameraInfo loadCameraCalibration( const std::string& filename, size_t width, size_t height )
{
  std::ifstream ifs( filename.c_str() );

  // read calibration matrix
  boost::array<double, 9> intrinsics;
  for(int i = 0; i < 9; ++i) {
    ifs >> intrinsics[i];
  }

  // create CameraInfo message
  sensor_msgs::CameraInfo camera_info;

  camera_info.height = height;
  camera_info.width = height;

  // The distortion model used. Supported models are listed in
  // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
  // simple model of radial and tangential distortion - is sufficent.
  camera_info.distortion_model = "plumb_bob";

  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
  camera_info.D = {0, 0, 0, 0, 0}; // TODO

  camera_info.K = intrinsics;

  camera_info.R = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  return camera_info;
}

void saveStream( cv::VideoCapture& capture, sensor_msgs::CameraInfo camera_info, const std::vector<ros::Time>& times, const std::string& frame_id, const std::string& topic, rosbag::Bag& bag )
{
  cv_bridge::CvImage ros_image;

  uint seq = 0;

  while ( capture.read( ros_image.image ) /*and seq < 500*/ )
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

    bag.write(topic + "/image_rect", times[ seq ], ros_image_msg);

    // create CameraInfo message

    //~ sensor_msgs::CameraInfo camera_info;
    camera_info.header.seq = seq;
    camera_info.header.stamp = times[ seq ];
    camera_info.header.frame_id = frame_id;

    bag.write(topic + "/camera_info", times[ seq ], camera_info);

    seq++;
  }
}

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
  int nFrames = (int) capture_left.get( CV_CAP_PROP_FRAME_COUNT );

  std::cout << "-- Loading " << nFrames << " frames of size " << width << "x" << height << std::endl;

  std::string left_calib = options["calib"].as<std::string>();
  sensor_msgs::CameraInfo cam_info_left = loadCameraCalibration( left_calib, width, height );

  std::cout << "Parsing timestamps ..." << std::endl;

  // TODO hardcoded
  double framerate = 1.0 / 30;

  std::vector<ros::Time> timestamps = options.count("timestamps") ? loadTimestamps( options["timestamps"].as<std::string>() ) : loadTimestamps( nFrames, framerate );

  std::cout << "ts: " << timestamps.size() << std::endl;
  assert( nFrames == timestamps.size() );

  // open a new bag file in write mode
  rosbag::Bag bag;

  bag.open(options["output"].as<std::string>(), rosbag::bagmode::Write);

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

  if ( options.count("poses") ) {

    std::cout << "Parsing ground truth poses ..." << std::endl;
    
    std::ifstream ifs(options["poses"].as<std::string>().c_str());
    ifs >> boost::tuples::set_delimiter(',') >> boost::tuples::set_open(' ') >> boost::tuples::set_close(' ');

    size_t seq = 0;
    while(ifs.good()) {
      // TODO: poses with timestamps case
      boost::tuple<float, float, float, float> tuple;
      ifs >> tuple;

      // write odometry message
      nav_msgs::Odometry odo_msg;
      odo_msg.header.frame_id = "odom";
      odo_msg.child_frame_id = "base_link";
      odo_msg.pose.pose.position.x = tuple.get<0>();
      odo_msg.pose.pose.position.y = tuple.get<1>();
      odo_msg.pose.pose.position.z = 0;
      odo_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(tuple.get<3>());
      bag.write("/robot/odometry", timestamps[ seq ], odo_msg);

      // write tf message
      geometry_msgs::TransformStamped tmsg;
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
      
      bag.write("/tf", timestamps[ seq ], tf_msg);

      seq++;
    }
  }

  bag.close();
}
