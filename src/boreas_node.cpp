#include <Eigen/Dense>
#include <boreas/boreas_node.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <tf2_ros/qos.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>
#include <climits>
#include <cmath>
#include <regex>

namespace boreas
{
using namespace std::chrono_literals;

BoreasNode::BoreasNode() : Node("boreas")
{
  data_path_ = declare_parameter<std::string>("data_path");
  output_bag_path_ = declare_parameter<std::string>("output_bag", "boreas_bag");
  pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/boreas/pointcloud");
  image_topic_ = declare_parameter<std::string>("image_topic", "/boreas/image/compressed");
  camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/boreas/camera_info");
  image_compression_format_ = declare_parameter<std::string>("image_compression_format", "png");
  jpeg_quality_ = declare_parameter<int>("jpeg_quality", 90);
  lidar_downsample_stride_ = declare_parameter<int>("lidar_downsample_stride", 1);
  storage_id_ = declare_parameter<std::string>("storage_id", "sqlite3");
  bag_duration_sec_ = declare_parameter<double>("bag_duration_sec", 0.0);
  tf_static_topic_ = declare_parameter<std::string>("tf_static_topic", "/tf_static");
  tf_topic_ = declare_parameter<std::string>("tf_topic", "/tf");
  clock_topic_ = declare_parameter<std::string>("clock_topic", "/clock");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  base_link_frame_ = declare_parameter<std::string>("base_link_frame", "base_link");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "lidar");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "camera");
  base_link_to_lidar_yaw_deg_ = declare_parameter<double>("base_link_to_lidar_yaw_deg", -45.0);
  use_precomputed_static_tf_ = declare_parameter<bool>("use_precomputed_static_tf", true);
  lidar_ground_truth_topic_ =
    declare_parameter<std::string>("lidar_ground_truth_topic", "/boreas/ground_truth/lidar_odom");
  camera_ground_truth_topic_ =
    declare_parameter<std::string>("camera_ground_truth_topic", "/boreas/ground_truth/camera_odom");
  gnss_topic_ = declare_parameter<std::string>("gnss_topic", "/boreas/gnss/fix");
  ground_truth_parent_frame_ = declare_parameter<std::string>("ground_truth_parent_frame", "enu");
  write_ground_truth_ = declare_parameter<bool>("write_ground_truth", true);
  write_tf_ = declare_parameter<bool>("write_tf", true);
  write_gnss_ = declare_parameter<bool>("write_gnss", true);

  lidar_data_path_ = data_path_ + "/lidar";
  camera_data_path_ = data_path_ + "/camera";
  applanix_data_path_ = data_path_ + "/applanix";
  camera_to_lidar_ = data_path_ + "/calib/T_camera_lidar.txt";

  init_bag_writer();
  init_static_transforms();
  load_camera_info();
  load_ground_truth_data();

  callback_group1_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callback_group3_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer1_ = this->create_wall_timer(1s, std::bind(&BoreasNode::function1, this), callback_group1_);
  timer2_ = this->create_wall_timer(1s, std::bind(&BoreasNode::function2, this), callback_group2_);
  timer3_ = this->create_wall_timer(1s, std::bind(&BoreasNode::function3, this), callback_group3_);
}

BoreasNode::~BoreasNode()
{
  if (writer_) {
    writer_->close();
  }
}

void BoreasNode::init_bag_writer()
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = output_bag_path_;
  storage_options.storage_id = storage_id_;

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(storage_options);
  RCLCPP_INFO(
    get_logger(), "Opened rosbag writer at: %s (storage=%s)", output_bag_path_.c_str(),
    storage_id_.c_str());
}

void BoreasNode::register_bag_topics()
{
  // Match tf2_ros::StaticBroadcasterQoS: KeepLast(1), Reliable, Transient Local
  const rclcpp::QoS tf_static_qos = tf2_ros::StaticBroadcasterQoS();

  rosbag2_storage::TopicMetadata tf_static_meta;
  tf_static_meta.name = tf_static_topic_;
  tf_static_meta.type = "tf2_msgs/msg/TFMessage";
  tf_static_meta.serialization_format = "cdr";
  tf_static_meta.offered_qos_profiles = {tf_static_qos};
  writer_->create_topic(tf_static_meta);

  if (write_tf_) {
    // Match tf2_ros::DynamicBroadcasterQoS: KeepLast(100), Reliable, Volatile
    const rclcpp::QoS tf_qos = tf2_ros::DynamicBroadcasterQoS();

    rosbag2_storage::TopicMetadata tf_meta;
    tf_meta.name = tf_topic_;
    tf_meta.type = "tf2_msgs/msg/TFMessage";
    tf_meta.serialization_format = "cdr";
    tf_meta.offered_qos_profiles = {tf_qos};
    writer_->create_topic(tf_meta);
  }
}

void BoreasNode::remove_slash_and_bin(std::string & in)
{
  if (!in.empty() && in.front() == '/') {
    in.erase(0, 1);
  }

  size_t pos = in.rfind(".bin");
  if (pos != std::string::npos) {
    in.erase(pos, 4);
  }
}

void BoreasNode::clean_string(std::string & in)
{
  size_t pos = in.find(data_path_);
  if (pos != std::string::npos) {
    in.erase(pos, data_path_.length());
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

sensor_msgs::msg::PointCloud2 BoreasNode::load_lidar_pointcloud(const std::string & path)
{
  constexpr uint32_t k_fields = 6;
  constexpr uint32_t k_point_step = k_fields * sizeof(float);

  std::ifstream ifs(path, std::ios::binary | std::ios::ate);
  if (!ifs.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open lidar file: %s", path.c_str());
    return sensor_msgs::msg::PointCloud2();
  }

  const std::streamsize file_size = ifs.tellg();
  if (file_size <= 0 || file_size % k_point_step != 0) {
    RCLCPP_ERROR(get_logger(), "Invalid lidar file size: %s", path.c_str());
    return sensor_msgs::msg::PointCloud2();
  }

  const uint32_t num_points = static_cast<uint32_t>(file_size / k_point_step);
  const uint32_t stride = static_cast<uint32_t>(std::max(1, lidar_downsample_stride_));
  const uint32_t out_points = (num_points + stride - 1) / stride;
  const float scan_time_sec = static_cast<float>(getStampFromPath(path) * 1.0e-6);

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = lidar_frame_;
  msg.height = 1;
  msg.width = out_points;
  msg.is_dense = false;
  msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32, "channel", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(out_points);

  std::vector<char> buffer(static_cast<size_t>(file_size));
  ifs.seekg(0, std::ios::beg);
  ifs.read(buffer.data(), file_size);

  const float * src = reinterpret_cast<const float *>(buffer.data());

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
  sensor_msgs::PointCloud2Iterator<float> iter_r(msg, "channel");
  sensor_msgs::PointCloud2Iterator<float> iter_t(msg, "time");

  for (uint32_t in_i = 0, out_i = 0; out_i < out_points; in_i += stride, ++out_i) {
    const float * point = src + (in_i * k_fields);
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    *iter_i = point[3];
    *iter_r = point[4];
    *iter_t = point[5] + scan_time_sec;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_i;
    ++iter_r;
    ++iter_t;
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

bool BoreasNode::load_camera_to_lidar_calibration(
  const std::string & path, Eigen::Isometry3d & camera_to_lidar)
{
  const auto matrix = read_matrix_from_file(path);
  if (matrix.size() != 4 || matrix[0].size() != 4) {
    RCLCPP_ERROR(
      get_logger(), "Invalid transformation matrix size in %s. Expected 4x4.", path.c_str());
    return false;
  }

  camera_to_lidar = matrix4_to_isometry(matrix);
  RCLCPP_INFO(get_logger(), "Loaded T_camera_lidar from %s", path.c_str());
  return true;
}

void BoreasNode::init_static_transforms()
{
  Eigen::Isometry3d base_link_to_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d base_link_to_camera = Eigen::Isometry3d::Identity();

  if (use_precomputed_static_tf_) {
    const auto bl_lidar_translation = declare_parameter<std::vector<double>>(
      "base_link_to_lidar.translation", std::vector<double>{});
    const auto bl_lidar_rotation =
      declare_parameter<std::vector<double>>("base_link_to_lidar.rotation", std::vector<double>{});
    const auto bl_camera_translation = declare_parameter<std::vector<double>>(
      "base_link_to_camera.translation", std::vector<double>{});
    const auto bl_camera_rotation =
      declare_parameter<std::vector<double>>("base_link_to_camera.rotation", std::vector<double>{});

    if (
      bl_lidar_translation.size() == 3 && bl_lidar_rotation.size() == 4 &&
      bl_camera_translation.size() == 3 && bl_camera_rotation.size() == 4) {
      base_link_to_lidar = params_to_isometry(bl_lidar_translation, bl_lidar_rotation);
      base_link_to_camera = params_to_isometry(bl_camera_translation, bl_camera_rotation);
      RCLCPP_INFO(get_logger(), "Loaded precomputed static transforms from parameters");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "use_precomputed_static_tf=true but params missing; computing from calibration file");
      use_precomputed_static_tf_ = false;
    }
  }

  if (!use_precomputed_static_tf_) {
    Eigen::Isometry3d camera_to_lidar;
    if (!load_camera_to_lidar_calibration(camera_to_lidar_, camera_to_lidar)) {
      RCLCPP_ERROR(get_logger(), "Failed to load camera-lidar calibration; static TF unavailable");
      return;
    }

    base_link_to_lidar = yaw_only_isometry(base_link_to_lidar_yaw_deg_);
    base_link_to_camera = base_link_to_lidar * camera_to_lidar.inverse();
    RCLCPP_INFO(
      get_logger(), "Computed static transforms from %s with %.1f deg yaw offset",
      camera_to_lidar_.c_str(), base_link_to_lidar_yaw_deg_);
  }

  base_link_to_lidar_isometry_ = base_link_to_lidar;
  base_link_to_lidar_transform_ =
    isometry_to_transform(base_link_to_lidar, base_link_frame_, lidar_frame_);
  base_link_to_camera_transform_ =
    isometry_to_transform(base_link_to_camera, base_link_frame_, camera_frame_);
  static_transforms_ready_ = true;

  const auto & t = base_link_to_camera.translation();
  const Eigen::Quaterniond q(base_link_to_camera.rotation());
  RCLCPP_INFO(
    get_logger(),
    "Static TF: %s -> %s (yaw %.1f deg), %s -> %s (t=[%.3f, %.3f, %.3f], q=[%.4f, %.4f, %.4f, "
    "%.4f])",
    base_link_frame_.c_str(), lidar_frame_.c_str(), base_link_to_lidar_yaw_deg_,
    base_link_frame_.c_str(), camera_frame_.c_str(), t.x(), t.y(), t.z(), q.x(), q.y(), q.z(),
    q.w());
}

void BoreasNode::write_static_transform_to_bag()
{
  if (!static_transforms_ready_) {
    RCLCPP_ERROR(get_logger(), "Static transforms not ready; skipping /tf_static");
    return;
  }

  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  tf2_msgs::msg::TFMessage tf_msg;

  base_link_to_lidar_transform_.header.stamp = stamp;
  base_link_to_camera_transform_.header.stamp = stamp;
  tf_msg.transforms.push_back(base_link_to_lidar_transform_);
  tf_msg.transforms.push_back(base_link_to_camera_transform_);

  writer_->write(tf_msg, tf_static_topic_, stamp);
  RCLCPP_INFO(
    get_logger(), "Wrote static transforms (%s -> %s, %s -> %s) on %s", base_link_frame_.c_str(),
    lidar_frame_.c_str(), base_link_frame_.c_str(), camera_frame_.c_str(),
    tf_static_topic_.c_str());
}

bool BoreasNode::load_camera_info()
{
  camera_info_msg_.header.frame_id = camera_frame_;
  camera_info_msg_.width = declare_parameter<int>("width");
  camera_info_msg_.height = declare_parameter<int>("height");
  camera_info_msg_.k[0] = declare_parameter<double>("k0");
  camera_info_msg_.k[2] = declare_parameter<double>("k2");
  camera_info_msg_.k[4] = declare_parameter<double>("k4");
  camera_info_msg_.k[5] = declare_parameter<double>("k5");
  camera_info_msg_.k[8] = 1.0;

  camera_info_msg_.p[0] = camera_info_msg_.k[0];
  camera_info_msg_.p[2] = camera_info_msg_.k[2];
  camera_info_msg_.p[5] = camera_info_msg_.k[4];
  camera_info_msg_.p[6] = camera_info_msg_.k[5];
  camera_info_msg_.p[10] = 1.0;

  return true;
}

bool BoreasNode::load_ground_truth_data()
{
  if (!write_ground_truth_ && !write_gnss_ && !write_tf_) {
    ground_truth_ready_ = true;
    return true;
  }

  const std::string lidar_pose_path = applanix_data_path_ + "/lidar_poses.csv";
  const std::string camera_pose_path = applanix_data_path_ + "/camera_poses.csv";
  const std::string gnss_path = applanix_data_path_ + "/gps_post_process.csv";

  if (write_ground_truth_ || write_tf_) {
    if (!load_pose_csv(lidar_pose_path, lidar_poses_)) {
      RCLCPP_WARN(get_logger(), "Failed to load lidar ground truth: %s", lidar_pose_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu lidar ground truth poses", lidar_poses_.size());
    }
  }

  if (write_ground_truth_) {
    if (!load_pose_csv(camera_pose_path, camera_poses_)) {
      RCLCPP_WARN(get_logger(), "Failed to load camera ground truth: %s", camera_pose_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu camera ground truth poses", camera_poses_.size());
    }
  }

  if (write_gnss_) {
    if (!load_gnss_csv(gnss_path, gnss_samples_)) {
      RCLCPP_WARN(get_logger(), "Failed to load GNSS data: %s", gnss_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu GNSS samples", gnss_samples_.size());
    }
  }

  ground_truth_ready_ = true;
  return true;
}

void BoreasNode::write_ground_truth_to_bag(
  long long timestamp_us, const rclcpp::Time & stamp, const std::string & topic,
  const std::string & child_frame, const std::unordered_map<long long, BoreasPose> & poses)
{
  const BoreasPose * pose = lookup_pose(poses, timestamp_us);
  if (pose == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No ground truth pose for timestamp %lld", timestamp_us);
    return;
  }

  nav_msgs::msg::Odometry odom = pose_to_odometry(*pose, ground_truth_parent_frame_, child_frame);
  odom.header.stamp = stamp;
  writer_->write(odom, topic, stamp);
}

void BoreasNode::write_map_to_base_link_tf(long long timestamp_us, const rclcpp::Time & stamp)
{
  if (!write_tf_) {
    return;
  }

  const BoreasPose * pose = lookup_pose(lidar_poses_, timestamp_us);
  if (pose == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No lidar pose for map->base_link TF at timestamp %lld",
      timestamp_us);
    return;
  }

  const Eigen::Isometry3d map_to_lidar = pose_to_isometry(*pose);
  const Eigen::Isometry3d map_to_base_link = map_to_lidar * base_link_to_lidar_isometry_.inverse();

  geometry_msgs::msg::TransformStamped transform =
    isometry_to_transform(map_to_base_link, map_frame_, base_link_frame_);
  transform.header.stamp = stamp;

  tf2_msgs::msg::TFMessage tf_msg;
  tf_msg.transforms.push_back(transform);
  writer_->write(tf_msg, tf_topic_, stamp);
}

void BoreasNode::write_gnss_sample_to_bag(const GnssSample & sample, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::NavSatFix fix = gnss_to_nav_sat_fix(sample);
  fix.header.frame_id = ground_truth_parent_frame_;
  fix.header.stamp = stamp;
  writer_->write(fix, gnss_topic_, stamp);
}

void BoreasNode::write_lidar_frame(long long timestamp_us, const std::string & frame_path)
{
  const rclcpp::Time stamp = id_to_stamp(timestamp_us);

  sensor_msgs::msg::PointCloud2 pc_msg = load_lidar_pointcloud(frame_path);
  if (pc_msg.width == 0) {
    return;
  }

  pc_msg.header.stamp = stamp;
  writer_->write(pc_msg, pointcloud_topic_, stamp);

  if (write_ground_truth_) {
    write_ground_truth_to_bag(
      timestamp_us, stamp, lidar_ground_truth_topic_, lidar_frame_, lidar_poses_);
  }

  write_map_to_base_link_tf(timestamp_us, stamp);

  write_clock_to_bag(stamp);
  messages_written_++;
  lidar_frames_written_++;
  if (first_lidar_timestamp_us_ == 0) {
    first_lidar_timestamp_us_ = timestamp_us;
  }
  last_lidar_timestamp_us_ = timestamp_us;
}

void BoreasNode::write_camera_frame(long long timestamp_us, const std::string & frame_path)
{
  const rclcpp::Time stamp = id_to_stamp(timestamp_us);

  sensor_msgs::msg::CompressedImage::SharedPtr image_msg = read_compressed_image(frame_path);
  if (!image_msg) {
    return;
  }

  image_msg->header.frame_id = camera_frame_;
  image_msg->header.stamp = stamp;
  writer_->write(*image_msg, image_topic_, stamp);

  camera_info_msg_.header.stamp = stamp;
  writer_->write(camera_info_msg_, camera_info_topic_, stamp);

  if (write_ground_truth_) {
    write_ground_truth_to_bag(
      timestamp_us, stamp, camera_ground_truth_topic_, camera_frame_, camera_poses_);
  }

  write_clock_to_bag(stamp);
  messages_written_++;
}

size_t BoreasNode::count_total_messages() const
{
  const auto count_in_window = [this](const auto & messages) {
    return static_cast<size_t>(std::count_if(
      messages.begin(), messages.end(),
      [this](const auto & entry) { return is_within_bag_duration(entry.first); }));
  };

  size_t total = count_in_window(lidar_sorted_vec_) + count_in_window(camera_sorted_vec_);
  if (write_gnss_) {
    total += static_cast<size_t>(std::count_if(
      gnss_samples_.begin(), gnss_samples_.end(),
      [this](const GnssSample & sample) { return is_within_bag_duration(sample.timestamp_us); }));
  }
  return total;
}

bool BoreasNode::is_within_bag_duration(long long timestamp_us) const
{
  if (bag_end_timestamp_us_ <= 0) {
    return true;
  }
  return timestamp_us <= bag_end_timestamp_us_;
}

void BoreasNode::write_streams_chronologically()
{
  size_t lidar_idx = 0;
  size_t camera_idx = 0;
  size_t gnss_idx = 0;
  ProgressBar progress_bar(total_messages_);

  while (rclcpp::ok()) {
    const bool has_lidar = lidar_idx < lidar_sorted_vec_.size();
    const bool has_camera = camera_idx < camera_sorted_vec_.size();
    const bool has_gnss = write_gnss_ && gnss_idx < gnss_samples_.size();

    if (!has_lidar && !has_camera && !has_gnss) {
      break;
    }

    long long next_timestamp_us = LLONG_MAX;
    if (has_lidar) {
      next_timestamp_us = std::min(next_timestamp_us, lidar_sorted_vec_[lidar_idx].first);
    }
    if (has_camera) {
      next_timestamp_us = std::min(next_timestamp_us, camera_sorted_vec_[camera_idx].first);
    }
    if (has_gnss) {
      next_timestamp_us = std::min(next_timestamp_us, gnss_samples_[gnss_idx].timestamp_us);
    }

    if (!is_within_bag_duration(next_timestamp_us)) {
      break;
    }

    if (has_lidar && lidar_sorted_vec_[lidar_idx].first == next_timestamp_us) {
      write_lidar_frame(next_timestamp_us, lidar_sorted_vec_[lidar_idx].second);
      progress_bar.update(messages_written_);
      lidar_idx++;
    }

    if (has_camera && camera_sorted_vec_[camera_idx].first == next_timestamp_us) {
      write_camera_frame(next_timestamp_us, camera_sorted_vec_[camera_idx].second);
      progress_bar.update(messages_written_);
      camera_idx++;
    }

    if (has_gnss && gnss_samples_[gnss_idx].timestamp_us == next_timestamp_us) {
      const rclcpp::Time stamp = id_to_stamp(next_timestamp_us);
      write_gnss_sample_to_bag(gnss_samples_[gnss_idx], stamp);
      write_clock_to_bag(stamp);
      messages_written_++;
      progress_bar.update(messages_written_);
      gnss_idx++;
    }
  }

  progress_bar.finish(messages_written_);
}

void BoreasNode::function1()
{
  if (!lidar_data_ready_) {
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

    RCLCPP_INFO_STREAM(get_logger(), "lidar data prepared");
    lidar_data_ready_ = true;
  }
}

void BoreasNode::function2()
{
  if (!camera_data_ready_) {
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

    RCLCPP_INFO_STREAM(get_logger(), "camera data prepared");
    camera_data_ready_ = true;
  }
}

sensor_msgs::msg::CompressedImage::SharedPtr BoreasNode::read_compressed_image(
  const std::string & image_path)
{
  auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();

  if (image_compression_format_ == "png") {
    std::ifstream file(image_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open image: %s", image_path.c_str());
      return {};
    }

    const std::streamsize size = file.tellg();
    if (size <= 0) {
      RCLCPP_ERROR(get_logger(), "Empty image file: %s", image_path.c_str());
      return {};
    }

    file.seekg(0, std::ios::beg);
    msg->data.resize(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char *>(msg->data.data()), size)) {
      RCLCPP_ERROR(get_logger(), "Failed to read image: %s", image_path.c_str());
      return {};
    }

    msg->format = "png";
    return msg;
  }

  if (image_compression_format_ == "jpeg") {
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to load image: %s", image_path.c_str());
      return {};
    }

    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
    if (!cv::imencode(".jpg", image, msg->data, params)) {
      RCLCPP_ERROR(get_logger(), "Failed to encode JPEG: %s", image_path.c_str());
      return {};
    }

    msg->format = "jpeg";
    return msg;
  }

  RCLCPP_ERROR(
    get_logger(), "Unsupported image_compression_format '%s' (use 'png' or 'jpeg')",
    image_compression_format_.c_str());
  return {};
}

void BoreasNode::write_clock_to_bag(const rclcpp::Time & stamp)
{
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = stamp;
  writer_->write(clock_msg, clock_topic_, stamp);
}

void BoreasNode::function3()
{
  if (!done_) {
    if (camera_data_ready_ && lidar_data_ready_ && ground_truth_ready_) {
      RCLCPP_INFO_STREAM(get_logger(), "lidar  size:= " << lidar_sorted_vec_.size());
      RCLCPP_INFO_STREAM(get_logger(), "camera  size:= " << camera_sorted_vec_.size());

      if (lidar_sorted_vec_.empty() || camera_sorted_vec_.empty()) {
        RCLCPP_ERROR(get_logger(), "No lidar or camera frames found, aborting.");
        writer_->close();
        rclcpp::shutdown();
        return;
      }

      time_origin_us_ = std::min(lidar_sorted_vec_.front().first, camera_sorted_vec_.front().first);
      if (write_gnss_ && !gnss_samples_.empty()) {
        time_origin_us_ = std::min(time_origin_us_, gnss_samples_.front().timestamp_us);
      }
      if (bag_duration_sec_ > 0.0) {
        bag_end_timestamp_us_ =
          time_origin_us_ + static_cast<long long>(bag_duration_sec_ * 1'000'000.0);
        RCLCPP_INFO(
          get_logger(), "Bag duration cap: %.1f s (end timestamp %lld us)", bag_duration_sec_,
          bag_end_timestamp_us_);
      } else {
        bag_end_timestamp_us_ = 0;
      }
      RCLCPP_INFO(
        get_logger(), "Bag time origin set to %lld us (relative timestamps from 0)",
        time_origin_us_);
      register_bag_topics();
      write_static_transform_to_bag();

      total_messages_ = count_total_messages();
      RCLCPP_INFO(
        get_logger(),
        "Writing up to %zu messages (%zu lidar, %zu camera, %zu gnss in window) to %s",
        total_messages_,
        static_cast<size_t>(std::count_if(
          lidar_sorted_vec_.begin(), lidar_sorted_vec_.end(),
          [this](const auto & e) { return is_within_bag_duration(e.first); })),
        static_cast<size_t>(std::count_if(
          camera_sorted_vec_.begin(), camera_sorted_vec_.end(),
          [this](const auto & e) { return is_within_bag_duration(e.first); })),
        write_gnss_
          ? static_cast<size_t>(std::count_if(
              gnss_samples_.begin(), gnss_samples_.end(),
              [this](const GnssSample & s) { return is_within_bag_duration(s.timestamp_us); }))
          : 0,
        output_bag_path_.c_str());
      write_streams_chronologically();
      done_ = true;
      if (lidar_frames_written_ > 1) {
        const double duration_sec =
          static_cast<double>(last_lidar_timestamp_us_ - first_lidar_timestamp_us_) * 1e-6;
        const double lidar_hz = (lidar_frames_written_ - 1) / duration_sec;
        RCLCPP_INFO(
          get_logger(), "Lidar: %zu frames over %.1f s (%.2f Hz), downsample stride=%d",
          lidar_frames_written_, duration_sec, lidar_hz, lidar_downsample_stride_);
      }
      RCLCPP_INFO(
        get_logger(), "Finished writing %zu/%zu messages to %s", messages_written_, total_messages_,
        output_bag_path_.c_str());
      writer_->close();
      rclcpp::shutdown();
    }
  }
}

rclcpp::Time BoreasNode::id_to_stamp(long long int timestamp_us) const
{
  const long long relative_us = timestamp_us - time_origin_us_;
  const int64_t sec = relative_us / 1'000'000LL;
  const uint32_t nanosec = static_cast<uint32_t>((relative_us % 1'000'000LL) * 1'000LL);
  return rclcpp::Time(sec, nanosec, RCL_ROS_TIME);
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
