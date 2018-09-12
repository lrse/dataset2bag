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
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>

#include "PrettyIFStream.hpp"

namespace po = boost::program_options;

// Macros used to copy a cv::Mat into an array
#define CP_MAT_TO_ARRAY(m, a) { for(auto i=0; i<(m).rows; i++) for(auto j=0; j<(m).cols; j++) (a)[i*(m).cols + j] = (m).ptr<double>(i)[j]; }
#define CP_ARRAY_TO_MAT(a, m) { for(auto i=0; i<(m).rows; i++) for(auto j=0; j<(m).cols; j++) (m).ptr<double>(i)[j] = (a)[i*(m).cols + j]; }

struct CameraParameters
{
  // intrinsic parameters
  // 3x3 camera matrix
  cv::Mat K;
  // 5x1 distortion parameters
  cv::Mat d;

  // extrinsic parameters, rotation (3x3) and translation (3x1)
  // only make sens for the right camera of a stereo pair
  cv::Mat R, t;

	size_t width, height;
};

bool process_args(int argc, char** argv, po::variables_map& options)
{
  po::options_description options_description("Options");
  options_description.add_options()
    ("images,l", po::value<std::string>(), "glob pattern of images (e.g. \"img/*.png\") or video file from which to extract frames (for monocular case or left images of stereo pair)")

    ("images_right,r", po::value<std::string>(), "right images of stereo pair, same expected input")

		("compressed_images", po::value<std::string>(), "specify if, when reading images from directory, they should be saved in their original compressed format. Specify the compression format, either \"png\" or \"jpg\"")

    ("odometry", po::value<std::string>(), "path to file containing 2D odometry data")

    ("output,o", po::value<std::string>()->required(), "output bag file")

    ("calib,c", po::value<std::string>(), "left camera calibration parameters file")

    ("calib_right", po::value<std::string>(), "right camera calibration parameters file")

    ("timestamps,t", po::value<std::string>(), "path to file containing timestamps")

    ("imu", po::value<std::string>(), "path to IMU data file")
    ("groundtruth", po::value<std::string>(), "path to ground-truth data file")
    ("gt-with-covariance", "if specified, the ground-truth file contains covariance information")

		("chunk-size", po::value<int>(), "set chunk size in KB of bag file (default: 768)")

    //("static-transforms", po::value<std::string>(), "path to file containing static transforms between sensors")
    ("laser", po::value<std::string>(), "path to laser scans file")

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
	std::string line;

  while (std::getline(ifs, line))
	{
		std::stringstream ss(line);

		uint32_t sec, nsec;
    ss >> sec >> nsec;

		if (!ss) throw std::runtime_error("Invalid timestamp!");
    timestamps.push_back(ros::Time(sec, nsec));
  }

  return timestamps;
}

CameraParameters loadCameraCalibration( const std::string& filename )
{
  CameraParameters ret;

  std::ifstream ifs(filename.c_str());

	/* image size */
	ifs >> ret.width >> ret.height;

	boost::array<double, 9> intrinsics;
	for(int i = 0; i < 9; ++i) ifs >> intrinsics[i];
	ret.K = cv::Mat(3, 3, CV_64FC1);
	CP_ARRAY_TO_MAT(intrinsics, ret.K);

	boost::array<double, 5> dist_coefficients;
	for(int i = 0; i < 5; ++i) ifs >> dist_coefficients[i];
	ret.d = cv::Mat(5, 1, CV_64FC1);
	CP_ARRAY_TO_MAT(dist_coefficients, ret.d);

	boost::array<double, 9> rotation;
	for(int i = 0; i < 9; ++i) ifs >> rotation[i];
	ret.R = cv::Mat(3, 3, CV_64FC1);
	CP_ARRAY_TO_MAT(rotation, ret.R);

	boost::array<double, 3> translation;
	for(int i = 0; i < 3; ++i) ifs >> translation[i];
	//~ ret.t = cv::Mat(3, 1, CV_64FC1, translation.c_array());
	ret.t = cv::Mat(3, 1, CV_64FC1);
	CP_ARRAY_TO_MAT(translation, ret.t);

  if (!ifs) throw std::runtime_error("Error reading calibration file");

	std::cout << "Calibration:" << std::endl;
	std::cout << ret.width << " x " << ret.height << std::endl;
	std::cout << ret.K << std::endl;
	std::cout << ret.d << std::endl;
	std::cout << ret.R << std::endl;
	std::cout << ret.t << std::endl;

  return ret;
}

sensor_msgs::CameraInfo loadCameraInfo(const CameraParameters& params)
{
  // create CameraInfo message
  sensor_msgs::CameraInfo camera_info;

	camera_info.width = params.width;
  camera_info.height = params.height;
  camera_info.distortion_model = "plumb_bob";
  camera_info.D = std::vector<double>( params.d.begin<double>(), params.d.end<double>() );
  CP_MAT_TO_ARRAY(params.K, camera_info.K);
  camera_info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}; // Id

  /* build trivial P from K */
  cv::Mat P = params.K * cv::Mat::eye(3, 4, CV_64FC1);
  CP_MAT_TO_ARRAY(P, camera_info.P);

  return camera_info;
}

sensor_msgs::CameraInfo loadCameraInfo( const std::string& filename)
{
  CameraParameters params = loadCameraCalibration(filename);
  return loadCameraInfo(params);
}

void loadStereoCameraCalibration( const std::string& filename_left, const std::string& filename_right, sensor_msgs::CameraInfo& cam_info_left, sensor_msgs::CameraInfo& cam_info_right )
{
  CameraParameters params_left = loadCameraCalibration(filename_left);
  CameraParameters params_right = loadCameraCalibration(filename_right);

  cam_info_left = loadCameraInfo(params_left);
  cam_info_right = loadCameraInfo(params_right);

  // Compute rectification transforms for each head of the stereo camera
  cv::Mat R1, R2, P1, P2, Q;
  stereoRectify(params_left.K, params_left.d, params_right.K, params_right.d, cv::Size(params_left.width, params_left.height), params_right.R, params_right.t, R1, R2, P1, P2, Q);

	std::cout << "Stereo Calibration: " << std::endl;
	std::cout << P1 << std::endl;
	std::cout << P2 << std::endl;
	std::cout << R1 << std::endl;
	std::cout << R2 << std::endl;

  CP_MAT_TO_ARRAY(R1, cam_info_left.R);
  CP_MAT_TO_ARRAY(P1, cam_info_left.P);

  CP_MAT_TO_ARRAY(R2, cam_info_right.R);
  CP_MAT_TO_ARRAY(P2, cam_info_right.P);
}

void saveStream(cv::VideoCapture& capture, sensor_msgs::CameraInfo camera_info, const std::vector<ros::Time>& times, const std::string& frame_id, const std::string& topic, rosbag::Bag& bag)
{
  cv_bridge::CvImage ros_image;
  ProgressBar progress( 80 );

  uint seq = 0;
	size_t frame_count = (size_t)capture.get(CV_CAP_PROP_FRAME_COUNT);

  while (capture.read(ros_image.image) /*&& seq < 500*/)
  {
    //std::cout << "loading image " << seq << "/" << frame_count << std::endl;
    cv::cvtColor(ros_image.image, ros_image.image, CV_BGR2RGB);
    ros_image.encoding = "rgb8";

    sensor_msgs::ImagePtr ros_image_msg;

    // create image message

    ros_image_msg = ros_image.toImageMsg();
    ros_image_msg->header.seq = seq;
    ros_image_msg->header.frame_id = frame_id;

		if (times.empty()) {
			ros_image_msg->header.stamp = ros::Time().fromSec(capture.get(CV_CAP_PROP_POS_MSEC) * 1e-3);
		}
		else {
			ros_image_msg->header.stamp = times[seq];
		}

    bag.write(topic + "/image_raw", ros_image_msg->header.stamp, ros_image_msg);

    // create CameraInfo message

    //~ sensor_msgs::CameraInfo camera_info;
    camera_info.header.seq = seq;
    camera_info.header.stamp = ros_image_msg->header.stamp;
    camera_info.header.frame_id = frame_id;

    bag.write(topic + "/camera_info", ros_image_msg->header.stamp, camera_info);

    progress.drawbar( float(seq) / frame_count );
    seq++;
  }
}

void saveStream(const std::vector<boost::filesystem::directory_entry>& entries, sensor_msgs::CameraInfo camera_info, const std::vector<ros::Time>& times,
								const std::string& frame_id, const std::string& topic, const std::string& compression_format, rosbag::Bag& bag)
{
  cv_bridge::CvImage ros_image;
  ProgressBar progress(80);

  uint seq = 0;

	for (const auto& entry : entries)
  {
		if (compression_format.empty())
		{
			ros_image.image = cv::imread(entry.path().string());
			cv::cvtColor(ros_image.image, ros_image.image, CV_BGR2RGB);
			ros_image.encoding = "rgb8";

			sensor_msgs::ImagePtr ros_image_msg;
			ros_image_msg = ros_image.toImageMsg();
			ros_image_msg->header.seq = seq;
			ros_image_msg->header.stamp = times[ seq ];
			ros_image_msg->header.frame_id = frame_id;
			bag.write(topic + "/image_raw", times[ seq ], ros_image_msg);
		}
		else
		{
			std::ifstream image_file(entry.path().string(), std::ios::binary);
			sensor_msgs::CompressedImage compressed_image;
			compressed_image.header.seq = seq;
			compressed_image.format = (compression_format == "jpg" ? "jpeg" : "png");
			compressed_image.header.stamp = times[seq];
			compressed_image.header.frame_id = frame_id;
			compressed_image.data.assign(std::istreambuf_iterator<char>(image_file), std::istreambuf_iterator<char>());
			bag.write(topic + "/image_raw/compressed", times[ seq ], compressed_image);
		}

    camera_info.header.seq = seq;
    camera_info.header.stamp = times[ seq ];
    camera_info.header.frame_id = frame_id;

    bag.write(topic + "/camera_info", times[ seq ], camera_info);

    progress.drawbar( float(seq) / times.size() );
    seq++;
  }
}

void process_images(const std::string& images_parameter, const sensor_msgs::CameraInfo& cam_info, const std::vector<ros::Time>& timestamps,
										const std::string& frame, const std::string& topic, const std::string& compression_format, rosbag::Bag& bag)
{
	size_t frame_count;

	/* process input images */
	boost::filesystem::path images_path(images_parameter);
	if (boost::filesystem::is_regular_file(images_path))
	{
		/* parameter assumed to be path to video file */
		cv::VideoCapture capture(images_parameter);

    frame_count = (size_t) capture.get(CV_CAP_PROP_FRAME_COUNT);
		std::cout << "Read video with " << frame_count << " frames" << std::endl;
		if (!timestamps.empty() && frame_count != timestamps.size()) throw std::runtime_error("Number of frames does not match number of timestamps");

		saveStream(capture, cam_info, timestamps, frame, topic, bag);
	}
	else if	(boost::filesystem::is_directory(images_path))
	{
		std::vector<boost::filesystem::directory_entry> entries;
		std::copy(boost::filesystem::directory_iterator(images_path), boost::filesystem::directory_iterator(), back_inserter(entries));
		std::sort(entries.begin(), entries.end(),	[](const boost::filesystem::directory_entry& entry1, const boost::filesystem::directory_entry& entry2) -> bool {
		    return entry1.path().filename() < entry2.path().filename();
		});

		std::vector<boost::filesystem::directory_entry> filtered_entries(entries.size());
		auto end_it = std::remove_copy_if(entries.begin(), entries.end(), filtered_entries.begin(), [](const boost::filesystem::directory_entry& entry) -> bool { return !boost::filesystem::is_regular_file(entry.path()); });

		filtered_entries.resize(std::distance(filtered_entries.begin(), end_it));
		entries = filtered_entries;

		frame_count = entries.size();
		std::cout << "Found " << frame_count << " images in directory" << std::endl;
		if (frame_count != timestamps.size()) throw std::runtime_error("Number of frames does not match number of timestamps");

		saveStream(entries, cam_info, timestamps, frame, topic, compression_format, bag);
	}
	else {
		throw std::runtime_error("Invalid images parameter input");
	}
}

bool input_is_video(const std::string& parameter)
{
	boost::filesystem::path path(parameter);
	if (boost::filesystem::is_regular_file(path)) return true;
	else if (boost::filesystem::is_directory(path)) return false;
	else throw std::runtime_error("Invalid images parameter input");
}

void saveOdometry(const std::string& filename, rosbag::Bag& bag)
{
  pretty_ifstream ifs(80, filename, std::ios::in);

  size_t seq = 0;
	std::string line;
  while (std::getline(ifs, line))
  {
		std::stringstream ss(line);

    float x, y, yaw;
    uint32_t sec, nsec;

    ss >> sec >> nsec >> x >> y >> yaw;
		if (!ss) throw std::runtime_error("Invalid odometry entry!");

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

    ifs.drawbar();
    seq++;
  }
}

void saveIMU(const std::string& filename, rosbag::Bag& bag)
{
  pretty_ifstream ifs(80, filename, std::ios::in);

  size_t seq = 0;
	std::string line;
  while(std::getline(ifs, line))
  {
		std::stringstream ss(line);

    float acc[3], gyro[3], R[9];
    uint32_t sec, nsec;

    ss >> sec >> nsec;
		for (int i = 0; i < 3; i++) ss >> gyro[i];
    for (int i = 0; i < 3; i++) ss >> acc[i];
    for (int i = 0; i < 9; i++) ss >> R[i];

		if (!ss) throw std::runtime_error("Invalid IMU entry!");

    ros::Time stamp(sec, nsec);

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

		imu_msg.angular_velocity_covariance[0] = 0.001f;
		imu_msg.angular_velocity_covariance[1] = 0.0f;
		imu_msg.angular_velocity_covariance[2] = 0.0f;
		imu_msg.angular_velocity_covariance[3] = 0.0f;
		imu_msg.angular_velocity_covariance[4] = 0.001f;
		imu_msg.angular_velocity_covariance[5] = 0.0f;
		imu_msg.angular_velocity_covariance[6] = 0.0f;
		imu_msg.angular_velocity_covariance[7] = 0.0f;
		imu_msg.angular_velocity_covariance[8] = 0.001f;
		imu_msg.linear_acceleration_covariance = imu_msg.orientation_covariance = imu_msg.angular_velocity_covariance;

    tf::Quaternion Q;
    tf::Matrix3x3 Rmat(R[0], R[1], R[2],
											 R[3], R[4], R[5],
											 R[6], R[7], R[8]);
    Rmat.getRotation(Q);
    tf::quaternionTFToMsg(Q, imu_msg.orientation);

    bag.write("/imu", stamp, imu_msg);

    ifs.drawbar();
    seq++;
  }
}

void saveGT(const std::string& filename, rosbag::Bag& bag, bool with_covariance)
{
  pretty_ifstream ifs(80, filename, std::ios::in);

  size_t seq = 0;
	std::string line;
  while(std::getline(ifs, line))
  {
		std::stringstream ss(line);

    uint32_t sec, nsec;
    float x, y, theta;
    float cov[9];

    ss >> sec >> nsec;
    ss >> x >> y >> theta;
    if (with_covariance) for (int i = 0; i < 9; i++) ss >> cov[i];

		if (!ss) throw std::runtime_error("Invalid GT entry!");

		ros::Time stamp(sec, nsec);

		if (with_covariance)
		{
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

    seq++;
    ifs.drawbar();
  }
}

void saveLaser(const std::string& filename, rosbag::Bag& bag)
{
  pretty_ifstream ifs(80, filename, std::ios::in);

	std::string line;
	std::getline(ifs, line);
	std::stringstream ss_first(line);

  float delta_angle, range_min, range_max, angle_min, angle_max;
  size_t scan_count;
  ss_first >> delta_angle >> scan_count;
  ss_first >> range_min >> range_max;
  ss_first >> angle_min >> angle_max;

	if (!ss_first) throw std::runtime_error("Invalid laser header");
  //std::cout << "reading laser: " << delta_angle << " " << scan_count << " " << range_min << " " << range_max << " " << angle_min << " " << angle_max << std::endl;

  angle_min = angle_min * M_PI / 180.0;
  angle_max = angle_max * M_PI / 180.0;
  delta_angle = delta_angle * M_PI / 180.0;

  size_t seq = 0;
  while(std::getline(ifs, line))
	{
		std::stringstream ss(line);

		uint32_t sec, nsec;
    ss >> sec >> nsec;

		ros::Time stamp(sec, nsec);

    sensor_msgs::LaserScan laser_msg;
    laser_msg.header.stamp = stamp;
    laser_msg.header.seq = seq;
    laser_msg.header.frame_id = "laser";
    laser_msg.angle_increment = delta_angle;
    laser_msg.range_min = range_min;
    laser_msg.range_max = range_max;
    laser_msg.angle_min = angle_min;
    laser_msg.angle_max = angle_max;
    laser_msg.ranges.resize(scan_count);
		for (size_t i = 0; i < scan_count; i++) ss >> laser_msg.ranges[i];

		if (!ss) throw std::runtime_error("Invalid laser entry");

    bag.write("/scan", stamp, laser_msg);
    ifs.drawbar();
    seq++;
  }
}

int main(int argc, char** argv)
{
  po::variables_map options;

  if ( not process_args(argc, argv, options) )
    return EXIT_FAILURE;

  rosbag::Bag bag;

  bag.open(options["output"].as<std::string>(), rosbag::bagmode::Write);

	if (options.count("chunk-size")) {
		size_t chunk_size = options["chunk-size"].as<int>();
		if (chunk_size <= 0) throw std::runtime_error("Invalid chunk size");
		bag.setChunkThreshold(chunk_size * 1024);
	}

  if (options.count("odometry")) {
    std::cout << "Parsing odometry file..." << std::endl;
    saveOdometry(options["odometry"].as<std::string>(), bag);
  }

  if (options.count("imu")) {
    std::cout << "Parsing IMU data..." << std::endl;
    saveIMU(options["imu"].as<std::string>(), bag);
  }

  if (options.count("groundtruth")) {
    std::cout << "Parsing ground-truth data" << ( options.count("gt-with-covariance") ? " with covariance..." : "..." ) << std::endl;
    saveGT(options["groundtruth"].as<std::string>(), bag, options.count("gt-with-covariance"));
  }

  if (options.count("laser")) {
    std::cout << "Parsing laser data..." << std::endl;
    saveLaser(options["laser"].as<std::string>(), bag);
  }

  /** process images **/
  if (options.count("images") != 0)
  {
		bool is_video = input_is_video(options["images"].as<std::string>());
		if (!is_video && options.count("timestamps") == 0)
			throw std::runtime_error("You need to specify a timestamps file");

		std::string compression_format;
		if (!is_video && options.count("compressed_images")) {
			compression_format = options["compressed_images"].as<std::string>();

			if (compression_format != "jpg" && compression_format != "png") throw std::runtime_error("Only \"jpg\" or \"png\" compressed image formats are allowed");
		}
		if (options.count("calib") == 0 || (options.count("images_right") && !options.count("calib_right"))) throw std::runtime_error("Camera calibration is required");

		/* load camera calibrations */
		sensor_msgs::CameraInfo cam_info_left, cam_info_right;
		if (options.count("calib_right"))
    {
      std::cout << "Parsing stereo camera calibrations ..." << std::endl;
      std::string left_calib = options["calib"].as<std::string>();
      std::string right_calib = options["calib_right"].as<std::string>();
      loadStereoCameraCalibration(left_calib, right_calib, cam_info_left, cam_info_right);
    }
    else
    {
      std::cout << "Parsing camera calibration ..." << std::endl;
      std::string left_calib = options["calib"].as<std::string>();
      cam_info_left = loadCameraInfo(left_calib);
    }

		std::vector<ros::Time> timestamps;
		/* load timestamps from file */
		if (options.count("timestamps")) {
			std::cout << "Parsing timestamps ..." << std::endl;
			timestamps = loadTimestamps(options["timestamps"].as<std::string>());
			if (timestamps.empty()) throw std::runtime_error("No timestamps were found in timestamps file");
			std::cout << "loaded: " << timestamps.size() << " timestamps" << std::endl;
		}

		/* load images */
		if (options.count("images_right"))
		{
			std::cout << "Parsing left stereo images ..." << std::endl;
			process_images(options["images"].as<std::string>(), cam_info_left, timestamps, "camera_left", "/stereo/left", compression_format, bag);
			std::cout << "Parsing right stereo images ..." << std::endl;
			process_images(options["images_right"].as<std::string>(), cam_info_right, timestamps, "camera_right", "/stereo/right", compression_format, bag);
		}
		else
		{
			std::cout << "Parsing camera images ..." << std::endl;
			process_images(options["images"].as<std::string>(), cam_info_left, timestamps, "camera", "/camera", compression_format, bag);
		}
  }

  bag.close();
}
