#ifndef BOREAS_BOREAS_PARAMETERS_HPP_
#define BOREAS_BOREAS_PARAMETERS_HPP_

#include <sensor_msgs/msg/camera_info.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace boreas
{

struct DatasetPaths
{
  std::string data;
  std::string lidar;
  std::string camera;
  std::string applanix;
  std::string camera_to_lidar_calib;
  std::string applanix_to_lidar_calib;
};

struct TopicNames
{
  std::string pointcloud;
  std::string image;
  std::string camera_info;
  std::string tf_static;
  std::string tf;
  std::string clock;
  std::string lidar_ground_truth;
  std::string camera_ground_truth;
  std::string gnss;
  std::string map_projector_info;
};

struct FrameNames
{
  std::string map;
  std::string applanix;
  std::string base_link;
  std::string lidar;
  std::string camera_lidar;
  std::string lidar_odom;
  std::string camera_odom;
  std::string gnss;
  std::string ground_truth_parent;
};

struct ExportFlags
{
  bool write_ground_truth{true};
  bool write_tf{true};
  bool write_gnss{true};
  bool write_map_projector_info{true};
  bool use_precomputed_static_tf{true};
};

struct BoreasParameters
{
  DatasetPaths paths;
  TopicNames topics;
  FrameNames frames;
  ExportFlags export_flags;

  std::string output_bag;
  std::string storage_id{"sqlite3"};
  std::string image_compression_format{"png"};
  std::string map_projector_info_path;

  sensor_msgs::msg::CameraInfo camera_info;

  double bag_duration_sec{0.0};
  double base_link_to_lidar_yaw_deg{-45.0};
  int jpeg_quality{90};
  int lidar_downsample_stride{1};
};

BoreasParameters load_parameters(rclcpp::Node & node);

}  // namespace boreas

#endif  // BOREAS_BOREAS_PARAMETERS_HPP_
