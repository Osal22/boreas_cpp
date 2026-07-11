#ifndef BOREAS_SENSOR_LOADERS_HPP_
#define BOREAS_SENSOR_LOADERS_HPP_

#include <boreas/boreas_parameters.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>

namespace boreas
{

sensor_msgs::msg::PointCloud2 load_lidar_pointcloud(
  const std::string & path, const std::string & frame_id, int downsample_stride);

std::shared_ptr<sensor_msgs::msg::CompressedImage> load_compressed_image(
  const std::string & image_path, const std::string & compression_format, int jpeg_quality);

}  // namespace boreas

#endif  // BOREAS_SENSOR_LOADERS_HPP_
