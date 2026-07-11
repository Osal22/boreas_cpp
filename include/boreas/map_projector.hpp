#ifndef BOREAS_MAP_PROJECTOR_HPP_
#define BOREAS_MAP_PROJECTOR_HPP_

#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace boreas
{

struct MapProjectorLoadResult
{
  bool ready{false};
  autoware_map_msgs::msg::MapProjectorInfo info;
};

MapProjectorLoadResult load_map_projector_info(
  const std::string & yaml_path, bool required, const rclcpp::Logger & logger);

}  // namespace boreas

#endif  // BOREAS_MAP_PROJECTOR_HPP_
