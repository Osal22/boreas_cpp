#include <boreas/map_projector.hpp>

#include <autoware/map_projection_loader/map_projection_loader.hpp>

namespace boreas
{

MapProjectorLoadResult load_map_projector_info(
  const std::string & yaml_path, bool required, const rclcpp::Logger & logger)
{
  MapProjectorLoadResult result;

  if (!required) {
    return result;
  }

  if (yaml_path.empty()) {
    RCLCPP_WARN(
      logger,
      "map_projector_info_path is empty; GNSS map projection and MapProjectorInfo bag topic "
      "disabled");
    return result;
  }

  try {
    result.info = autoware::map_projection_loader::load_info_from_yaml(yaml_path);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Failed to load map projector info from %s: %s", yaml_path.c_str(), ex.what());
    return result;
  }

  if (result.info.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    RCLCPP_ERROR(logger, "map_projector_info is LOCAL type; cannot project GNSS to map coordinates");
    return result;
  }

  result.ready = true;
  RCLCPP_INFO(
    logger, "Loaded map projector info from %s (type=%s)", yaml_path.c_str(),
    result.info.projector_type.c_str());
  return result;
}

}  // namespace boreas
