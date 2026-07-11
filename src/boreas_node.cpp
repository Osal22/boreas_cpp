#include <boreas/boreas_node.hpp>
#include <boreas/sensor_loaders.hpp>

#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#include <tf2_ros/qos.hpp>

#include <algorithm>
#include <chrono>
#include <climits>
#include <filesystem>

namespace boreas
{
namespace
{
using namespace std::chrono_literals;
}  // namespace

BoreasNode::BoreasNode() : Node("boreas")
{
  params_ = load_parameters(*this);
  static_transforms_ = load_static_transforms(*this, params_);

  const bool needs_map_projector =
    params_.export_flags.write_tf || params_.export_flags.write_map_projector_info;
  map_projector_ = load_map_projector_info(
    params_.map_projector_info_path, needs_map_projector, get_logger());
  if (needs_map_projector && !map_projector_.ready) {
    RCLCPP_WARN(get_logger(), "Map projector not loaded; map->applanix TF will be skipped");
  }

  init_bag_writer();
  load_ground_truth_data();

  lidar_index_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  camera_index_callback_group_ =
    create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  export_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  lidar_index_timer_ = create_wall_timer(
    1s, std::bind(&BoreasNode::index_lidar_frames, this), lidar_index_callback_group_);
  camera_index_timer_ = create_wall_timer(
    1s, std::bind(&BoreasNode::index_camera_frames, this), camera_index_callback_group_);
  export_timer_ = create_wall_timer(
    1s, std::bind(&BoreasNode::export_bag_when_ready, this), export_callback_group_);
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
  storage_options.uri = params_.output_bag;
  storage_options.storage_id = params_.storage_id;

  writer_ = std::make_unique<rosbag2_cpp::Writer>();
  writer_->open(storage_options);
  RCLCPP_INFO(
    get_logger(), "Opened rosbag writer at: %s (storage=%s)", params_.output_bag.c_str(),
    params_.storage_id.c_str());
}

void BoreasNode::register_bag_topics()
{
  const rclcpp::QoS tf_static_qos = tf2_ros::StaticBroadcasterQoS();

  rosbag2_storage::TopicMetadata tf_static_meta;
  tf_static_meta.name = params_.topics.tf_static;
  tf_static_meta.type = "tf2_msgs/msg/TFMessage";
  tf_static_meta.serialization_format = "cdr";
  tf_static_meta.offered_qos_profiles = {tf_static_qos};
  writer_->create_topic(tf_static_meta);

  if (params_.export_flags.write_tf) {
    const rclcpp::QoS tf_qos = tf2_ros::DynamicBroadcasterQoS();

    rosbag2_storage::TopicMetadata tf_meta;
    tf_meta.name = params_.topics.tf;
    tf_meta.type = "tf2_msgs/msg/TFMessage";
    tf_meta.serialization_format = "cdr";
    tf_meta.offered_qos_profiles = {tf_qos};
    writer_->create_topic(tf_meta);
  }

  if (params_.export_flags.write_map_projector_info && map_projector_.ready) {
    rclcpp::QoS map_info_qos(rclcpp::KeepLast(1));
    map_info_qos.transient_local();
    map_info_qos.reliable();

    rosbag2_storage::TopicMetadata map_info_meta;
    map_info_meta.name = params_.topics.map_projector_info;
    map_info_meta.type = "autoware_map_msgs/msg/MapProjectorInfo";
    map_info_meta.serialization_format = "cdr";
    map_info_meta.offered_qos_profiles = {map_info_qos};
    writer_->create_topic(map_info_meta);
  }
}

bool BoreasNode::load_ground_truth_data()
{
  const auto & flags = params_.export_flags;
  if (!flags.write_ground_truth && !flags.write_gnss && !flags.write_tf) {
    ground_truth_ready_ = true;
    return true;
  }

  const std::string lidar_pose_path = params_.paths.applanix + "/lidar_poses.csv";
  const std::string camera_pose_path = params_.paths.applanix + "/camera_poses.csv";
  const std::string gnss_path = params_.paths.applanix + "/gps_post_process.csv";

  if (flags.write_ground_truth || flags.write_tf) {
    if (!load_pose_csv(lidar_pose_path, lidar_poses_)) {
      RCLCPP_WARN(get_logger(), "Failed to load lidar ground truth: %s", lidar_pose_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu lidar ground truth poses", lidar_poses_.size());
    }
  }

  if (flags.write_ground_truth || flags.write_tf) {
    if (!load_pose_csv(camera_pose_path, camera_poses_)) {
      RCLCPP_WARN(get_logger(), "Failed to load camera ground truth: %s", camera_pose_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu camera ground truth poses", camera_poses_.size());
    }
  }

  if (flags.write_gnss || flags.write_tf) {
    if (!load_gnss_csv(gnss_path, gnss_samples_)) {
      RCLCPP_WARN(get_logger(), "Failed to load GNSS data: %s", gnss_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Loaded %zu GNSS samples", gnss_samples_.size());
    }
  }

  ground_truth_ready_ = true;
  return true;
}

void BoreasNode::write_static_transform_to_bag()
{
  if (!static_transforms_.ready) {
    RCLCPP_ERROR(get_logger(), "Static transforms not ready; skipping /tf_static");
    return;
  }

  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  tf2_msgs::msg::TFMessage tf_msg;
  static_transforms_.applanix_to_base_link.header.stamp = stamp;
  static_transforms_.base_link_to_lidar.header.stamp = stamp;
  static_transforms_.base_link_to_camera_lidar.header.stamp = stamp;
  tf_msg.transforms.push_back(static_transforms_.applanix_to_base_link);
  tf_msg.transforms.push_back(static_transforms_.base_link_to_lidar);
  tf_msg.transforms.push_back(static_transforms_.base_link_to_camera_lidar);

  writer_->write(tf_msg, params_.topics.tf_static, stamp);
  RCLCPP_INFO(
    get_logger(), "Wrote static transforms (%s -> %s, %s -> %s, %s -> %s) on %s",
    params_.frames.applanix.c_str(), params_.frames.base_link.c_str(), params_.frames.base_link.c_str(),
    params_.frames.lidar.c_str(), params_.frames.base_link.c_str(),
    params_.frames.camera_lidar.c_str(), params_.topics.tf_static.c_str());
}

void BoreasNode::write_map_projector_info_to_bag()
{
  if (!params_.export_flags.write_map_projector_info || !map_projector_.ready) {
    return;
  }

  const rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
  writer_->write(map_projector_.info, params_.topics.map_projector_info, stamp);
  RCLCPP_INFO(
    get_logger(), "Wrote MapProjectorInfo on %s (projector_type=%s)",
    params_.topics.map_projector_info.c_str(), map_projector_.info.projector_type.c_str());
}

void BoreasNode::write_tf_to_bag(const geometry_msgs::msg::TransformStamped & transform)
{
  tf2_msgs::msg::TFMessage tf_msg;
  tf_msg.transforms.push_back(transform);
  writer_->write(tf_msg, params_.topics.tf, transform.header.stamp);
}

void BoreasNode::write_ground_truth_to_bag(
  const int64_t timestamp_us, const rclcpp::Time & stamp, const std::string & topic,
  const std::string & child_frame, const BoreasPose & pose)
{
  if (!map_projector_.ready) {
    return;
  }

  Eigen::Isometry3d T_map_sensor;
  const Eigen::Isometry3d T_enu_sensor = pose_to_isometry(pose);
  if (!compute_map_sensor_pose(timestamp_us, pose, T_enu_sensor, T_map_sensor)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No GNSS sample for map-frame ground truth at %ld",
      timestamp_us);
    return;
  }

  nav_msgs::msg::Odometry odom =
    isometry_to_odometry(T_map_sensor, params_.frames.map, child_frame);
  odom.header.stamp = stamp;
  writer_->write(odom, topic, stamp);
}

bool BoreasNode::compute_map_sensor_pose(
  const int64_t timestamp_us, const BoreasPose & /*pose*/,
  const Eigen::Isometry3d & T_enu_sensor, Eigen::Isometry3d & T_map_sensor) const
{
  if (!map_projector_.ready || gnss_samples_.empty()) {
    return false;
  }

  const GnssSample gnss_sample = interpolate_gnss(gnss_samples_, timestamp_us);
  T_map_sensor =
    enu_sensor_pose_to_map_isometry(T_enu_sensor, gnss_sample, map_projector_.info);
  return true;
}

Eigen::Isometry3d BoreasNode::map_sensor_to_odom_transform(
  const Eigen::Isometry3d & T_map_sensor, std::optional<Eigen::Isometry3d> & odom_reference) const
{
  if (!odom_reference.has_value()) {
    odom_reference = T_map_sensor;
  }
  return T_map_sensor.inverse() * odom_reference.value();
}

void BoreasNode::write_map_to_applanix_tf(const int64_t timestamp_us, const rclcpp::Time & stamp)
{
  if (!params_.export_flags.write_tf || !map_projector_.ready) {
    return;
  }

  if (gnss_samples_.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No GNSS samples for map->applanix TF at timestamp %ld",
      timestamp_us);
    return;
  }

  const GnssSample gnss_sample = interpolate_gnss(gnss_samples_, timestamp_us);
  const Eigen::Isometry3d map_to_applanix =
    gnss_sample_to_map_isometry(gnss_sample, map_projector_.info);

  geometry_msgs::msg::TransformStamped transform =
    isometry_to_transform(map_to_applanix, params_.frames.map, params_.frames.applanix);
  transform.header.stamp = stamp;
  write_tf_to_bag(transform);
}

void BoreasNode::write_lidar_odom_tf(const int64_t timestamp_us, const rclcpp::Time & stamp)
{
  if (!params_.export_flags.write_tf || !map_projector_.ready) {
    return;
  }

  const BoreasPose * pose = lookup_pose(lidar_poses_, timestamp_us);
  if (pose == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "No lidar pose for lidar->lidar_odom TF at timestamp %ld",
      timestamp_us);
    return;
  }

  Eigen::Isometry3d T_map_lidar;
  if (!compute_map_sensor_pose(timestamp_us, *pose, pose_to_isometry(*pose), T_map_lidar)) {
    return;
  }

  std::optional<Eigen::Isometry3d> odom_reference = lidar_odom_reference_;
  const Eigen::Isometry3d T_lidar_lidar_odom =
    map_sensor_to_odom_transform(T_map_lidar, odom_reference);
  lidar_odom_reference_ = odom_reference;
  geometry_msgs::msg::TransformStamped transform = isometry_to_transform(
    T_lidar_lidar_odom, params_.frames.lidar, params_.frames.lidar_odom);
  transform.header.stamp = stamp;
  write_tf_to_bag(transform);
}

void BoreasNode::write_camera_odom_tf(const int64_t timestamp_us, const rclcpp::Time & stamp)
{
  if (!params_.export_flags.write_tf || !map_projector_.ready) {
    return;
  }

  const BoreasPose * pose = lookup_pose(camera_poses_, timestamp_us);
  if (pose == nullptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No camera pose for camera_lidar->camera_odom TF at timestamp %ld", timestamp_us);
    return;
  }

  Eigen::Isometry3d T_map_camera;
  if (!compute_map_sensor_pose(timestamp_us, *pose, pose_to_isometry(*pose), T_map_camera)) {
    return;
  }

  std::optional<Eigen::Isometry3d> odom_reference = camera_odom_reference_;
  const Eigen::Isometry3d T_camera_lidar_camera_odom =
    map_sensor_to_odom_transform(T_map_camera, odom_reference);
  camera_odom_reference_ = odom_reference;
  geometry_msgs::msg::TransformStamped transform = isometry_to_transform(
    T_camera_lidar_camera_odom, params_.frames.camera_lidar, params_.frames.camera_odom);
  transform.header.stamp = stamp;
  write_tf_to_bag(transform);
}

void BoreasNode::write_gnss_sample_to_bag(const GnssSample & sample, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::NavSatFix fix = gnss_to_nav_sat_fix(sample);
  fix.header.frame_id = params_.frames.gnss;
  fix.header.stamp = stamp;
  writer_->write(fix, params_.topics.gnss, stamp);
}

void BoreasNode::write_lidar_frame(const int64_t timestamp_us, const std::string & frame_path)
{
  const rclcpp::Time stamp = id_to_stamp(timestamp_us);

  sensor_msgs::msg::PointCloud2 pc_msg = load_lidar_pointcloud(
    frame_path, params_.frames.lidar, params_.lidar_downsample_stride);
  if (pc_msg.width == 0) {
    RCLCPP_ERROR(get_logger(), "Failed to load lidar file: %s", frame_path.c_str());
    return;
  }

  pc_msg.header.stamp = stamp;
  writer_->write(pc_msg, params_.topics.pointcloud, stamp);

  if (params_.export_flags.write_ground_truth) {
    const BoreasPose * pose = lookup_pose(lidar_poses_, timestamp_us);
    if (pose != nullptr) {
      write_ground_truth_to_bag(
        timestamp_us, stamp, params_.topics.lidar_ground_truth, params_.frames.lidar, *pose);
    }
  }

  write_map_to_applanix_tf(timestamp_us, stamp);
  write_lidar_odom_tf(timestamp_us, stamp);
  write_clock_to_bag(stamp);

  messages_written_++;
  lidar_frames_written_++;
  if (first_lidar_timestamp_us_ == 0) {
    first_lidar_timestamp_us_ = timestamp_us;
  }
  last_lidar_timestamp_us_ = timestamp_us;
}

void BoreasNode::write_camera_frame(const int64_t timestamp_us, const std::string & frame_path)
{
  const rclcpp::Time stamp = id_to_stamp(timestamp_us);

  const auto image_msg = load_compressed_image(
    frame_path, params_.image_compression_format, params_.jpeg_quality);
  if (!image_msg) {
    RCLCPP_ERROR(get_logger(), "Failed to load image: %s", frame_path.c_str());
    return;
  }

  image_msg->header.frame_id = params_.frames.camera_lidar;
  image_msg->header.stamp = stamp;
  writer_->write(*image_msg, params_.topics.image, stamp);

  auto camera_info = params_.camera_info;
  camera_info.header.stamp = stamp;
  writer_->write(camera_info, params_.topics.camera_info, stamp);

  if (params_.export_flags.write_ground_truth) {
    const BoreasPose * pose = lookup_pose(camera_poses_, timestamp_us);
    if (pose != nullptr) {
      write_ground_truth_to_bag(
        timestamp_us, stamp, params_.topics.camera_ground_truth, params_.frames.camera_lidar,
        *pose);
    }
  }

  write_camera_odom_tf(timestamp_us, stamp);
  write_clock_to_bag(stamp);
  messages_written_++;
}

void BoreasNode::write_clock_to_bag(const rclcpp::Time & stamp)
{
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = stamp;
  writer_->write(clock_msg, params_.topics.clock, stamp);
}

size_t BoreasNode::count_total_messages() const
{
  const auto count_in_window = [this](const auto & messages) {
    return static_cast<size_t>(std::count_if(
      messages.begin(), messages.end(),
      [this](const auto & entry) { return is_within_bag_duration(entry.first); }));
  };

  size_t total = count_in_window(lidar_frames_) + count_in_window(camera_frames_);
  if (params_.export_flags.write_gnss) {
    total += static_cast<size_t>(std::count_if(
      gnss_samples_.begin(), gnss_samples_.end(),
      [this](const GnssSample & sample) {
        return is_within_bag_duration(sample.timestamp_us);
      }));
  }
  return total;
}

bool BoreasNode::is_within_bag_duration(const int64_t timestamp_us) const
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
    const bool has_lidar = lidar_idx < lidar_frames_.size();
    const bool has_camera = camera_idx < camera_frames_.size();
    const bool has_gnss = params_.export_flags.write_gnss && gnss_idx < gnss_samples_.size();

    if (!has_lidar && !has_camera && !has_gnss) {
      break;
    }

    int64_t next_timestamp_us = LLONG_MAX;
    if (has_lidar) {
      next_timestamp_us = std::min(next_timestamp_us, lidar_frames_[lidar_idx].first);
    }
    if (has_camera) {
      next_timestamp_us = std::min(next_timestamp_us, camera_frames_[camera_idx].first);
    }
    if (has_gnss) {
      next_timestamp_us = std::min(next_timestamp_us, gnss_samples_[gnss_idx].timestamp_us);
    }

    if (!is_within_bag_duration(next_timestamp_us)) {
      break;
    }

    if (has_lidar && lidar_frames_[lidar_idx].first == next_timestamp_us) {
      write_lidar_frame(next_timestamp_us, lidar_frames_[lidar_idx].second);
      progress_bar.update(messages_written_);
      lidar_idx++;
    }

    if (has_camera && camera_frames_[camera_idx].first == next_timestamp_us) {
      write_camera_frame(next_timestamp_us, camera_frames_[camera_idx].second);
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

void BoreasNode::index_lidar_frames()
{
  if (lidar_index_ready_) {
    return;
  }

  try {
    lidar_frames_ = index_dataset_directory(
      params_.paths.lidar, params_.paths.data, "bin");
  } catch (const std::filesystem::filesystem_error & err) {
    RCLCPP_ERROR(get_logger(), "Failed to index lidar directory: %s", err.what());
    return;
  }

  RCLCPP_INFO(get_logger(), "Indexed %zu lidar frames", lidar_frames_.size());
  lidar_index_ready_ = true;
}

void BoreasNode::index_camera_frames()
{
  if (camera_index_ready_) {
    return;
  }

  try {
    camera_frames_ = index_dataset_directory(
      params_.paths.camera, params_.paths.data, "png");
  } catch (const std::filesystem::filesystem_error & err) {
    RCLCPP_ERROR(get_logger(), "Failed to index camera directory: %s", err.what());
    return;
  }

  RCLCPP_INFO(get_logger(), "Indexed %zu camera frames", camera_frames_.size());
  camera_index_ready_ = true;
}

void BoreasNode::export_bag_when_ready()
{
  if (export_done_ || !camera_index_ready_ || !lidar_index_ready_ || !ground_truth_ready_) {
    return;
  }

  if (lidar_frames_.empty() || camera_frames_.empty()) {
    RCLCPP_ERROR(get_logger(), "No lidar or camera frames found, aborting.");
    writer_->close();
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "lidar size: %zu", lidar_frames_.size());
  RCLCPP_INFO(get_logger(), "camera size: %zu", camera_frames_.size());

  time_origin_us_ = std::min(lidar_frames_.front().first, camera_frames_.front().first);
  if (params_.export_flags.write_gnss && !gnss_samples_.empty()) {
    time_origin_us_ = std::min(time_origin_us_, gnss_samples_.front().timestamp_us);
  }

  if (params_.bag_duration_sec > 0.0) {
    bag_end_timestamp_us_ =
      time_origin_us_ + static_cast<int64_t>(params_.bag_duration_sec * 1'000'000.0);
    RCLCPP_INFO(
      get_logger(), "Bag duration cap: %.1f s (end timestamp %ld us)", params_.bag_duration_sec,
      bag_end_timestamp_us_);
  } else {
    bag_end_timestamp_us_ = 0;
  }

  RCLCPP_INFO(
    get_logger(), "Bag time origin set to %ld us (relative timestamps from 0)", time_origin_us_);

  register_bag_topics();
  write_static_transform_to_bag();
  write_map_projector_info_to_bag();

  total_messages_ = count_total_messages();
  RCLCPP_INFO(
    get_logger(), "Writing up to %zu messages (%zu lidar, %zu camera, %zu gnss in window) to %s",
    total_messages_,
    static_cast<size_t>(std::count_if(
      lidar_frames_.begin(), lidar_frames_.end(),
      [this](const auto & entry) { return is_within_bag_duration(entry.first); })),
    static_cast<size_t>(std::count_if(
      camera_frames_.begin(), camera_frames_.end(),
      [this](const auto & entry) { return is_within_bag_duration(entry.first); })),
    params_.export_flags.write_gnss
      ? static_cast<size_t>(std::count_if(
          gnss_samples_.begin(), gnss_samples_.end(),
          [this](const GnssSample & sample) {
            return is_within_bag_duration(sample.timestamp_us);
          }))
      : 0,
    params_.output_bag.c_str());

  write_streams_chronologically();
  export_done_ = true;

  if (lidar_frames_written_ > 1) {
    const double duration_sec =
      static_cast<double>(last_lidar_timestamp_us_ - first_lidar_timestamp_us_) * 1e-6;
    const double lidar_hz = (lidar_frames_written_ - 1) / duration_sec;
    RCLCPP_INFO(
      get_logger(), "Lidar: %zu frames over %.1f s (%.2f Hz), downsample stride=%d",
      lidar_frames_written_, duration_sec, lidar_hz, params_.lidar_downsample_stride);
  }

  RCLCPP_INFO(
    get_logger(), "Finished writing %zu/%zu messages to %s", messages_written_, total_messages_,
    params_.output_bag.c_str());
  writer_->close();
  rclcpp::shutdown();
}

rclcpp::Time BoreasNode::id_to_stamp(const int64_t timestamp_us) const
{
  const int64_t relative_us = timestamp_us - time_origin_us_;
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
