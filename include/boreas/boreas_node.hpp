#ifndef BOREADS_NODE_HPP_
#define BOREADS_NODE_HPP_

#include <boreas/boreas.hpp>
#include <boreas/ground_truth.hpp>
#include <boreas/progress_bar.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace boreas
{
class BoreasNode : public rclcpp::Node
{
public:
  BoreasNode();
  ~BoreasNode();

private:
  void clean_string(std::string & in);
  void remove_slash_and_bin(std::string & in);
  long long int path_to_int(std::string & in);
  long long int path_to_int_cam(std::string & in);
  std::shared_ptr<sensor_msgs::msg::CompressedImage> read_compressed_image(
    const std::string & image_path);

  sensor_msgs::msg::PointCloud2 load_lidar_pointcloud(const std::string & path);
  std::vector<std::vector<double>> read_matrix_from_file(const std::string & filename);
  bool load_camera_to_lidar_calibration(
    const std::string & path, Eigen::Isometry3d & camera_to_lidar);
  void init_static_transforms();
  bool load_camera_info();
  bool load_ground_truth_data();
  void init_bag_writer();
  void register_bag_topics();
  void write_static_transform_to_bag();
  void write_map_to_base_link_tf(long long timestamp_us, const rclcpp::Time & stamp);
  void write_clock_to_bag(const rclcpp::Time & stamp);
  void write_ground_truth_to_bag(
    long long timestamp_us, const rclcpp::Time & stamp, const std::string & topic,
    const std::string & child_frame, const std::unordered_map<long long, BoreasPose> & poses);
  void write_gnss_sample_to_bag(const GnssSample & sample, const rclcpp::Time & stamp);
  void write_lidar_frame(long long timestamp_us, const std::string & frame_path);
  void write_camera_frame(long long timestamp_us, const std::string & frame_path);
  size_t count_total_messages() const;
  void write_streams_chronologically();
  bool is_within_bag_duration(long long timestamp_us) const;

  rclcpp::Time id_to_stamp(long long int timestamp_us) const;

  void function1();
  void function2();
  void function3();

  std::string data_path_;
  std::string lidar_data_path_;
  std::string camera_data_path_;
  std::string applanix_data_path_;
  std::string camera_to_lidar_;
  std::string output_bag_path_;
  std::string pointcloud_topic_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string tf_static_topic_;
  std::string tf_topic_;
  std::string clock_topic_;
  std::string map_frame_;
  std::string base_link_frame_;
  std::string lidar_frame_;
  std::string camera_frame_;
  std::string lidar_ground_truth_topic_;
  std::string camera_ground_truth_topic_;
  std::string gnss_topic_;
  std::string ground_truth_parent_frame_;
  std::string image_compression_format_{"png"};
  int jpeg_quality_{90};
  int lidar_downsample_stride_{2};
  std::string storage_id_{"sqlite3"};
  double base_link_to_lidar_yaw_deg_{-45.0};
  bool use_precomputed_static_tf_{true};
  double bag_duration_sec_{0.0};
  long long int bag_end_timestamp_us_{0};
  long long int time_origin_us_{0};
  long long int first_lidar_timestamp_us_{0};
  long long int last_lidar_timestamp_us_{0};
  size_t lidar_frames_written_{0};
  bool write_ground_truth_{true};
  bool write_tf_{true};
  bool write_gnss_{true};

  sensor_msgs::msg::CameraInfo camera_info_msg_;
  Eigen::Isometry3d base_link_to_lidar_isometry_{Eigen::Isometry3d::Identity()};
  geometry_msgs::msg::TransformStamped base_link_to_lidar_transform_;
  geometry_msgs::msg::TransformStamped base_link_to_camera_transform_;
  bool static_transforms_ready_{false};

  std::unordered_map<long long, BoreasPose> lidar_poses_;
  std::unordered_map<long long, BoreasPose> camera_poses_;
  std::vector<GnssSample> gnss_samples_;
  bool ground_truth_ready_{false};

  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::TimerBase::SharedPtr timer3_;

  rclcpp::CallbackGroup::SharedPtr callback_group1_;
  rclcpp::CallbackGroup::SharedPtr callback_group2_;
  rclcpp::CallbackGroup::SharedPtr callback_group3_;

  std::unordered_map<long long int, std::string> camera_sorted_;
  std::vector<std::pair<long long int, std::string>> camera_sorted_vec_;

  std::unordered_map<long long int, std::string> lidar_sorted_;
  std::vector<std::pair<long long int, std::string>> lidar_sorted_vec_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  bool lidar_data_ready_{false};
  bool camera_data_ready_{false};
  bool done_{false};
  size_t messages_written_{0};
  size_t total_messages_{0};
};
}  // namespace boreas
#endif  // BOREADS_NODE_HPP_
