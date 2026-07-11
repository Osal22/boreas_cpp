#include <boreas/boreas_parameters.hpp>

#include <rclcpp/rclcpp.hpp>

namespace boreas
{

BoreasParameters load_parameters(rclcpp::Node & node)
{
  BoreasParameters params;

  params.paths.data = node.declare_parameter<std::string>("data_path");
  params.output_bag = node.declare_parameter<std::string>("output_bag", "boreas_bag");
  params.storage_id = node.declare_parameter<std::string>("storage_id", "sqlite3");
  params.bag_duration_sec = node.declare_parameter<double>("bag_duration_sec", 0.0);
  params.lidar_downsample_stride = node.declare_parameter<int>("lidar_downsample_stride", 1);

  params.topics.pointcloud =
    node.declare_parameter<std::string>("pointcloud_topic", "/boreas/pointcloud");
  params.topics.image =
    node.declare_parameter<std::string>("image_topic", "/boreas/image/compressed");
  params.topics.camera_info =
    node.declare_parameter<std::string>("camera_info_topic", "/boreas/camera_info");
  params.topics.tf_static = node.declare_parameter<std::string>("tf_static_topic", "/tf_static");
  params.topics.tf = node.declare_parameter<std::string>("tf_topic", "/tf");
  params.topics.clock = node.declare_parameter<std::string>("clock_topic", "/clock");
  params.topics.lidar_ground_truth =
    node.declare_parameter<std::string>("lidar_ground_truth_topic", "/boreas/ground_truth/lidar_odom");
  params.topics.camera_ground_truth = node.declare_parameter<std::string>(
    "camera_ground_truth_topic", "/boreas/ground_truth/camera_odom");
  params.topics.gnss = node.declare_parameter<std::string>("gnss_topic", "/boreas/gnss/fix");
  params.topics.map_projector_info =
    node.declare_parameter<std::string>("map_projector_info_topic", "/map/map_projector_info");

  params.frames.map = node.declare_parameter<std::string>("map_frame", "map");
  params.frames.applanix = node.declare_parameter<std::string>("applanix_frame", "applanix");
  params.frames.base_link = node.declare_parameter<std::string>("base_link_frame", "base_link");
  params.frames.lidar = node.declare_parameter<std::string>("lidar_frame", "lidar");
  params.frames.camera_lidar =
    node.declare_parameter<std::string>("camera_lidar_frame", "camera_lidar");
  params.frames.lidar_odom =
    node.declare_parameter<std::string>("lidar_odom_frame", "lidar_odom");
  params.frames.camera_odom =
    node.declare_parameter<std::string>("camera_odom_frame", "camera_odom");
  params.frames.gnss = node.declare_parameter<std::string>("gnss_frame", "applanix");
  params.frames.ground_truth_parent =
    node.declare_parameter<std::string>("ground_truth_parent_frame", "map");

  params.map_projector_info_path =
    node.declare_parameter<std::string>("map_projector_info_path", "");
  params.base_link_to_lidar_yaw_deg =
    node.declare_parameter<double>("base_link_to_lidar_yaw_deg", -45.0);
  params.export_flags.use_precomputed_static_tf =
    node.declare_parameter<bool>("use_precomputed_static_tf", true);
  params.export_flags.write_map_projector_info =
    node.declare_parameter<bool>("write_map_projector_info", true);
  params.export_flags.write_ground_truth =
    node.declare_parameter<bool>("write_ground_truth", true);
  params.export_flags.write_tf = node.declare_parameter<bool>("write_tf", true);
  params.export_flags.write_gnss = node.declare_parameter<bool>("write_gnss", true);

  params.image_compression_format =
    node.declare_parameter<std::string>("image_compression_format", "png");
  params.jpeg_quality = node.declare_parameter<int>("jpeg_quality", 90);

  params.paths.lidar = params.paths.data + "/lidar";
  params.paths.camera = params.paths.data + "/camera";
  params.paths.applanix = params.paths.data + "/applanix";
  params.paths.camera_to_lidar_calib = params.paths.data + "/calib/T_camera_lidar.txt";
  params.paths.applanix_to_lidar_calib = params.paths.data + "/calib/T_applanix_lidar.txt";

  params.camera_info.header.frame_id = params.frames.camera_lidar;
  params.camera_info.width = node.declare_parameter<int>("width");
  params.camera_info.height = node.declare_parameter<int>("height");
  params.camera_info.k[0] = node.declare_parameter<double>("k0");
  params.camera_info.k[2] = node.declare_parameter<double>("k2");
  params.camera_info.k[4] = node.declare_parameter<double>("k4");
  params.camera_info.k[5] = node.declare_parameter<double>("k5");
  params.camera_info.k[8] = 1.0;
  params.camera_info.p[0] = params.camera_info.k[0];
  params.camera_info.p[2] = params.camera_info.k[2];
  params.camera_info.p[5] = params.camera_info.k[4];
  params.camera_info.p[6] = params.camera_info.k[5];
  params.camera_info.p[10] = 1.0;

  return params;
}

}  // namespace boreas
