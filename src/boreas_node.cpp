#include <Eigen/Dense>
#include <boreas/boreas_node.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <boreas/csv.h>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <regex>
#include <thread>
namespace boreas
{
using namespace std::chrono_literals;

BoreasNode::BoreasNode() : Node("boreas")
{
  data_path_ = declare_parameter<std::string>("data_path");
  pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/pointcloud", 10);
  camera_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image", 10);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("~/camera_info", 10);
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  lidar_data_path_ = data_path_ + "/lidar";
  camera_data_path_ = data_path_ + "/camera";
  camera_to_lidar_ = data_path_ + "/calib/T_camera_lidar.txt";

  publish_transform_op(camera_to_lidar_);
  load_camera_info();

  callback_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // timer1_ = this->create_wall_timer(1s, std::bind(&BoreasNode::function1, this),
  // callback_group1_); timer2_ = this->create_wall_timer(1s, std::bind(&BoreasNode::function2,
  // this), callback_group2_);
  timer3_ = this->create_wall_timer(1s, std::bind(&BoreasNode::function3, this), callback_group3_);

  writer_ = std::make_unique<rosbag2_cpp::Writer>();

  writer_->open("boreas_bag");
}

void BoreasNode::remove_slash_and_bin(std::string & in)
{
  if (!in.empty() && in.front() == '/') {
    in.erase(0, 1);
  }

  // Remove trailing ".bin"
  size_t pos = in.rfind(".bin");
  if (pos != std::string::npos) {
    in.erase(pos, 4);  // ".bin" has 4 characters
  }
}

void BoreasNode::clean_string(std::string & in)
{
  size_t pos = in.find(data_path_);
  if (pos != std::string::npos) {
    in.erase(pos, data_path_.length());  // Remove the substring
  }
}

long long int BoreasNode::path_to_int(std::string & in)
{
  long long int number{0};
  clean_string(in);
  std::regex number_regex(R"(/(\d+)\.bin$)");
  std::smatch match;
  if (std::regex_search(in, match, number_regex)) {
    std::string match_str = match.str();
    remove_slash_and_bin(match_str);
    number = std::stoll(match_str);
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "no match is found");
    return 0;
  }
  return number;
}

long long int BoreasNode::path_to_int_cam(std::string & in)
{
  long long int number{0};
  clean_string(in);
  std::regex number_regex(R"(/(\d+)\.png$)");
  std::smatch match;
  if (std::regex_search(in, match, number_regex)) {
    std::string match_str = match.str();
    remove_slash_and_bin(match_str);
    number = std::stoll(match_str);
  } else {
    RCLCPP_INFO_STREAM(get_logger(), "no match is found");
    return 0;
  }
  return number;
}

sensor_msgs::msg::PointCloud2 BoreasNode::eigen_to_pointcloud(const Eigen::MatrixXd & pc)
{
  sensor_msgs::msg::PointCloud2 msg;

  // Assume pc has 6 columns: x, y, z, intensity, channel, time
  size_t num_points = pc.rows();
  msg.header.frame_id = "lidar";  // Set appropriate frame
  msg.header.stamp = now();       // Set appropriate frame

  msg.height = 1;  // Organized if >1, unorganized if 1
  msg.width = num_points;
  msg.is_dense = false;  // True if no NaNs
  msg.is_bigendian = false;

  // Define the fields in PointCloud2
  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");  // Adds x, y, z
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32, "channel", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32);

  modifier.resize(num_points);

  // Use iterators to fill data
  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
  sensor_msgs::PointCloud2Iterator<float> iter_r(msg, "channel");
  sensor_msgs::PointCloud2Iterator<float> iter_c(msg, "time");

  for (size_t i = 0; i < num_points;
       ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i, ++iter_r, ++iter_c) {
    // float z_val = static_cast<float>(pc(i, 2));
    // if (z_val < -1.2) {
    *iter_x = static_cast<float>(pc(i, 0));
    *iter_y = static_cast<float>(pc(i, 1));
    *iter_z = static_cast<float>(pc(i, 2));
    *iter_i = static_cast<float>(pc(i, 3));
    *iter_r = static_cast<float>(pc(i, 4));
    *iter_c = static_cast<float>(pc(i, 5));
    // }
  }

  return msg;
}

std::vector<std::vector<double>> BoreasNode::read_matrix_from_file(const std::string & filename)
{
  std::ifstream file(filename);
  std::vector<std::vector<double>> matrix;
  std::string line;

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::vector<double> row;
    double value;
    while (iss >> value) {
      row.push_back(value);
    }
    if (!row.empty()) {
      matrix.push_back(row);
    }
  }
  return matrix;
}

void BoreasNode::publish_transform(const std::vector<std::vector<double>> & matrix)
{
  if (matrix.size() != 4 || matrix[0].size() != 4) {
    RCLCPP_ERROR(this->get_logger(), "Invalid transformation matrix size. Expected 4x4.");
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = "camera";
  transform.child_frame_id = "lidar";

  // Translation (last column of the matrix)
  transform.transform.translation.x = matrix[0][3];
  transform.transform.translation.y = matrix[1][3];
  transform.transform.translation.z = matrix[2][3];

  // Extract rotation using transformation matrix
  Eigen::Matrix3d rotation_matrix;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) rotation_matrix(i, j) = matrix[i][j];

  Eigen::Quaterniond quat(rotation_matrix);
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();

  static_broadcaster_->sendTransform(transform);
  RCLCPP_INFO(this->get_logger(), "Published static transform.");
}

void BoreasNode::publish_transform_op(std::string & path)
{
  publish_transform(read_matrix_from_file(path));
}

bool BoreasNode::load_camera_info()
{
  camera_info_msg_.header.frame_id = "camera";  // Change as needed
  camera_info_msg_.header.stamp = now();
  // Image size
  camera_info_msg_.width = declare_parameter<int>("width");
  camera_info_msg_.height = declare_parameter<int>("height");
  // Intrinsic matrix (K)
  camera_info_msg_.k[0] = declare_parameter<double>("k0");  // fx
  camera_info_msg_.k[2] = declare_parameter<double>("k2");  // cx
  camera_info_msg_.k[4] = declare_parameter<double>("k4");  // fy
  camera_info_msg_.k[5] = declare_parameter<double>("k5");  // cy
  camera_info_msg_.k[8] = 1.0;                              // Identity

  // Projection matrix (P)
  camera_info_msg_.p[0] = camera_info_msg_.k[0];  // fx
  camera_info_msg_.p[2] = camera_info_msg_.k[2];  // cx
  camera_info_msg_.p[5] = camera_info_msg_.k[4];  // fy
  camera_info_msg_.p[6] = camera_info_msg_.k[5];  // cy
  camera_info_msg_.p[10] = 1.0;

  return true;
}

void BoreasNode::function1()
{
  // read lidar data
  try {
    for (const auto & entry : fs::directory_iterator(lidar_data_path_)) {
      std::string frame_path = entry.path().string();
      std::string frame_path_solid = entry.path().string();

      long long int frame_no = path_to_int(frame_path);
      lidar_sorted_[frame_no] = frame_path_solid;
    }
  } catch (const fs::filesystem_error & err) {
    RCLCPP_ERROR_STREAM(get_logger(), "lidar" << err.what());
  }
  lidar_sorted_vec_ =
    std::vector<std::pair<long long, std::string>>(lidar_sorted_.begin(), lidar_sorted_.end());

  std::sort(lidar_sorted_vec_.begin(), lidar_sorted_vec_.end());
  while (rclcpp::ok()) {
    for (auto frame_set : lidar_sorted_vec_) {
      std::string lidar_frame_path = frame_set.second;
      Eigen::MatrixXd pc;
      load_lidar(lidar_frame_path, pc);
      sensor_msgs::msg::PointCloud2 pc_msg;
      pc_msg = eigen_to_pointcloud(pc);
      std::lock_guard<std::mutex> lock(mtx);  // Automatically locks & unlocks

      // lidar_msg_sorted_vec_.push_back(std::make_pair(frame_set.first, pc_msg));
      // pc_pub_->publish(pc_msg);
      // camera_info_msg_.header.stamp = now();
      // camera_info_pub_->publish(camera_info_msg_);
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  RCLCPP_INFO_STREAM(get_logger(), "lidar data prepared");
}

void BoreasNode::function2()
{
  // read camera data
  try {
    for (const auto & entry : fs::directory_iterator(camera_data_path_)) {
      std::string frame_path = entry.path().string();
      std::string frame_path_solid = entry.path().string();
      long long int frame_no = path_to_int_cam(frame_path);
      camera_sorted_[frame_no] = frame_path_solid;
    }
  } catch (const fs::filesystem_error & err) {
    RCLCPP_ERROR_STREAM(get_logger(), "camera" << err.what());
  }
  camera_sorted_vec_ =
    std::vector<std::pair<long long, std::string>>(camera_sorted_.begin(), camera_sorted_.end());

  std::sort(camera_sorted_vec_.begin(), camera_sorted_vec_.end());
  while (rclcpp::ok()) {
    for (auto frame_set : camera_sorted_vec_) {
      std::string camera_frame_path = frame_set.second;
      sensor_msgs::msg::Image::SharedPtr image_msg;
      image_msg = read_image(camera_frame_path);
      image_msg->header.frame_id = "camera";
      image_msg->header.stamp = now();
      camera_pub_->publish(*image_msg);
      std::lock_guard<std::mutex> lock(mtx);  // Automatically locks & unlocks
    }
  }
  RCLCPP_INFO_STREAM(get_logger(), "camera data prepared");
}

sensor_msgs::msg::Image::SharedPtr BoreasNode::read_image(std::string & image_path)
{
  cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
  if (image.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path.c_str());
    return {};
  }
  sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
  return msg;
}

void BoreasNode::function3()
{
  io::CSVReader<2> in("/home/osal/Downloads/boreas-2021-04-08-12-44/applanix/ros_and_gps_time.csv");
  in.read_header(io::ignore_extra_column, "ROS Time", "GPS Time");
  long long int ros_time;
  float gps_time;
  while (in.read_row(ros_time, gps_time)) {
    long sec = ros_time / 1'000'000'000;
    long nanosec = ros_time % 1'000'000'000;
    rclcpp::Time time(sec, nanosec);
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = time;
    auto ser_clock_msg = serialize_message(clock_msg);
    std::lock_guard<std::mutex> lock(mtx);  // Automatically locks & unlocks
    writer_->write(
      std::make_shared<rclcpp::SerializedMessage>(ser_clock_msg), "/clock",
      "rosgraph_msgs/msg/Clock", time);
  }
}

template <typename MsgT>
rclcpp::SerializedMessage BoreasNode::serialize_message(const MsgT & msg)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<MsgT> serializer;
  serializer.serialize_message(&msg, &serialized_msg);
  return serialized_msg;
}

}  // namespace boreas

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<boreas::BoreasNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
