#include <boreas/static_transforms.hpp>

#include <boreas/calibration.hpp>
#include <boreas/ground_truth.hpp>

namespace boreas
{

StaticTransformSet load_static_transforms(rclcpp::Node & node, const BoreasParameters & params)
{
  StaticTransformSet transforms;
  const auto & logger = node.get_logger();

  Eigen::Isometry3d base_link_to_lidar = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d base_link_to_camera_lidar = Eigen::Isometry3d::Identity();
  bool use_precomputed = params.export_flags.use_precomputed_static_tf;

  if (use_precomputed) {
    const auto bl_lidar_translation = node.declare_parameter<std::vector<double>>(
      "base_link_to_lidar.translation", std::vector<double>{});
    const auto bl_lidar_rotation =
      node.declare_parameter<std::vector<double>>("base_link_to_lidar.rotation", std::vector<double>{});
    const auto bl_camera_lidar_translation = node.declare_parameter<std::vector<double>>(
      "base_link_to_camera_lidar.translation", std::vector<double>{});
    const auto bl_camera_lidar_rotation = node.declare_parameter<std::vector<double>>(
      "base_link_to_camera_lidar.rotation", std::vector<double>{});

    if (
      bl_lidar_translation.size() == 3 && bl_lidar_rotation.size() == 4 &&
      bl_camera_lidar_translation.size() == 3 && bl_camera_lidar_rotation.size() == 4) {
      base_link_to_lidar = params_to_isometry(bl_lidar_translation, bl_lidar_rotation);
      base_link_to_camera_lidar =
        params_to_isometry(bl_camera_lidar_translation, bl_camera_lidar_rotation);
      RCLCPP_INFO(logger, "Loaded precomputed static transforms from parameters");
    } else {
      RCLCPP_WARN(
        logger,
        "use_precomputed_static_tf=true but params missing; computing from calibration file");
      use_precomputed = false;
    }
  }

  Eigen::Isometry3d camera_to_lidar;
  if (!load_camera_to_lidar_calibration(params.paths.camera_to_lidar_calib, camera_to_lidar)) {
    RCLCPP_ERROR(logger, "Failed to load camera-lidar calibration; static TF unavailable");
    return transforms;
  }

  if (!use_precomputed) {
    base_link_to_lidar = yaw_only_isometry(params.base_link_to_lidar_yaw_deg);
    base_link_to_camera_lidar = base_link_to_lidar * camera_to_lidar.inverse();
    RCLCPP_INFO(
      logger, "Computed static transforms from %s with %.1f deg yaw offset",
      params.paths.camera_to_lidar_calib.c_str(), params.base_link_to_lidar_yaw_deg);
  }

  Eigen::Isometry3d applanix_to_lidar;
  if (!load_camera_to_lidar_calibration(params.paths.applanix_to_lidar_calib, applanix_to_lidar)) {
    RCLCPP_ERROR(
      logger, "Failed to load Applanix-lidar calibration from %s",
      params.paths.applanix_to_lidar_calib.c_str());
    return transforms;
  }

  const Eigen::Isometry3d applanix_to_base_link = applanix_to_lidar * base_link_to_lidar.inverse();
  const Eigen::Isometry3d applanix_to_camera_lidar = applanix_to_lidar * camera_to_lidar.inverse();

  transforms.applanix_to_lidar = applanix_to_lidar;
  transforms.applanix_to_camera_lidar = applanix_to_camera_lidar;
  transforms.applanix_to_base_link =
    isometry_to_transform(applanix_to_base_link, params.frames.applanix, params.frames.base_link);
  transforms.base_link_to_lidar =
    isometry_to_transform(base_link_to_lidar, params.frames.base_link, params.frames.lidar);
  transforms.base_link_to_camera_lidar = isometry_to_transform(
    base_link_to_camera_lidar, params.frames.base_link, params.frames.camera_lidar);
  transforms.ready = true;

  const auto & t_ap_bl = applanix_to_base_link.translation();
  RCLCPP_INFO(
    logger,
    "Static TF: %s -> %s (t=[%.3f, %.3f, %.3f]), %s -> %s (yaw %.1f deg), %s -> %s (from %s)",
    params.frames.applanix.c_str(), params.frames.base_link.c_str(), t_ap_bl.x(), t_ap_bl.y(),
    t_ap_bl.z(), params.frames.base_link.c_str(), params.frames.lidar.c_str(),
    params.base_link_to_lidar_yaw_deg, params.frames.base_link.c_str(),
    params.frames.camera_lidar.c_str(), params.paths.applanix_to_lidar_calib.c_str());

  return transforms;
}

}  // namespace boreas
