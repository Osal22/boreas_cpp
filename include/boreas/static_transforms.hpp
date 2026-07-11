#ifndef BOREAS_STATIC_TRANSFORMS_HPP_
#define BOREAS_STATIC_TRANSFORMS_HPP_

#include <boreas/boreas_parameters.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace boreas
{

struct StaticTransformSet
{
  bool ready{false};
  Eigen::Isometry3d applanix_to_lidar{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d applanix_to_camera_lidar{Eigen::Isometry3d::Identity()};
  geometry_msgs::msg::TransformStamped applanix_to_base_link;
  geometry_msgs::msg::TransformStamped base_link_to_lidar;
  geometry_msgs::msg::TransformStamped base_link_to_camera_lidar;
};

StaticTransformSet load_static_transforms(rclcpp::Node & node, const BoreasParameters & params);

}  // namespace boreas

#endif  // BOREAS_STATIC_TRANSFORMS_HPP_
