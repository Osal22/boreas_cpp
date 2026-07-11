#ifndef BOREAS_BOREAS_NODE_HPP_
#define BOREAS_BOREAS_NODE_HPP_

#include <boreas/boreas_parameters.hpp>
#include <boreas/dataset_index.hpp>
#include <boreas/ground_truth.hpp>
#include <boreas/map_projector.hpp>
#include <boreas/progress_bar.hpp>
#include <boreas/static_transforms.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace boreas
{

class BoreasNode : public rclcpp::Node
{
public:
  BoreasNode();
  ~BoreasNode() override;

private:
  void init_bag_writer();
  void register_bag_topics();
  bool load_ground_truth_data();

  void write_static_transform_to_bag();
  void write_map_projector_info_to_bag();
  void write_tf_to_bag(const geometry_msgs::msg::TransformStamped & transform);
  void write_map_to_applanix_tf(int64_t timestamp_us, const rclcpp::Time & stamp);
  void write_lidar_odom_tf(int64_t timestamp_us, const rclcpp::Time & stamp);
  void write_camera_odom_tf(int64_t timestamp_us, const rclcpp::Time & stamp);
  void write_clock_to_bag(const rclcpp::Time & stamp);
  void write_ground_truth_to_bag(
    int64_t timestamp_us, const rclcpp::Time & stamp, const std::string & topic,
    const std::string & child_frame, const BoreasPose & pose);
  bool compute_map_sensor_pose(
    int64_t timestamp_us, const BoreasPose & pose, const Eigen::Isometry3d & T_enu_sensor,
    Eigen::Isometry3d & T_map_sensor) const;
  Eigen::Isometry3d map_sensor_to_odom_transform(
    const Eigen::Isometry3d & T_map_sensor, std::optional<Eigen::Isometry3d> & odom_reference) const;
  void write_gnss_sample_to_bag(const GnssSample & sample, const rclcpp::Time & stamp);
  void write_lidar_frame(int64_t timestamp_us, const std::string & frame_path);
  void write_camera_frame(int64_t timestamp_us, const std::string & frame_path);

  size_t count_total_messages() const;
  void write_streams_chronologically();
  bool is_within_bag_duration(int64_t timestamp_us) const;
  rclcpp::Time id_to_stamp(int64_t timestamp_us) const;

  void index_lidar_frames();
  void index_camera_frames();
  void export_bag_when_ready();

  BoreasParameters params_;
  StaticTransformSet static_transforms_;
  MapProjectorLoadResult map_projector_;

  std::unordered_map<int64_t, BoreasPose> lidar_poses_;
  std::unordered_map<int64_t, BoreasPose> camera_poses_;
  std::vector<GnssSample> gnss_samples_;

  std::vector<TimestampedPath> lidar_frames_;
  std::vector<TimestampedPath> camera_frames_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  rclcpp::TimerBase::SharedPtr lidar_index_timer_;
  rclcpp::TimerBase::SharedPtr camera_index_timer_;
  rclcpp::TimerBase::SharedPtr export_timer_;

  rclcpp::CallbackGroup::SharedPtr lidar_index_callback_group_;
  rclcpp::CallbackGroup::SharedPtr camera_index_callback_group_;
  rclcpp::CallbackGroup::SharedPtr export_callback_group_;

  bool lidar_index_ready_{false};
  bool camera_index_ready_{false};
  bool ground_truth_ready_{false};
  bool export_done_{false};

  int64_t bag_end_timestamp_us_{0};
  int64_t time_origin_us_{0};
  int64_t first_lidar_timestamp_us_{0};
  int64_t last_lidar_timestamp_us_{0};

  size_t messages_written_{0};
  size_t total_messages_{0};
  size_t lidar_frames_written_{0};

  mutable std::optional<Eigen::Isometry3d> lidar_odom_reference_;
  mutable std::optional<Eigen::Isometry3d> camera_odom_reference_;
};

}  // namespace boreas

#endif  // BOREAS_BOREAS_NODE_HPP_
