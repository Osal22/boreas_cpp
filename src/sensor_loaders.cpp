#include <boreas/path_utils.hpp>
#include <boreas/sensor_loaders.hpp>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <fstream>

namespace boreas
{

sensor_msgs::msg::PointCloud2 load_lidar_pointcloud(
  const std::string & path, const std::string & frame_id, int downsample_stride)
{
  constexpr uint32_t k_fields = 6;
  constexpr uint32_t k_point_step = k_fields * sizeof(float);

  std::ifstream ifs(path, std::ios::binary | std::ios::ate);
  if (!ifs.is_open()) {
    return sensor_msgs::msg::PointCloud2();
  }

  const std::streamsize file_size = ifs.tellg();
  if (file_size <= 0 || file_size % k_point_step != 0) {
    return sensor_msgs::msg::PointCloud2();
  }

  const uint32_t num_points = static_cast<uint32_t>(file_size / k_point_step);
  const uint32_t stride = static_cast<uint32_t>(std::max(1, downsample_stride));
  const uint32_t out_points = (num_points + stride - 1) / stride;
  const float scan_time_sec = static_cast<float>(timestamp_us_from_path(path) * 1.0e-6);

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame_id;
  msg.height = 1;
  msg.width = out_points;
  msg.is_dense = false;
  msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.setPointCloud2Fields(
    6, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1,
    sensor_msgs::msg::PointField::FLOAT32, "channel", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(out_points);

  std::vector<char> buffer(static_cast<size_t>(file_size));
  ifs.seekg(0, std::ios::beg);
  ifs.read(buffer.data(), file_size);

  const float * src = reinterpret_cast<const float *>(buffer.data());

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_i(msg, "intensity");
  sensor_msgs::PointCloud2Iterator<float> iter_r(msg, "channel");
  sensor_msgs::PointCloud2Iterator<float> iter_t(msg, "time");

  for (uint32_t in_i = 0, out_i = 0; out_i < out_points; in_i += stride, ++out_i) {
    const float * point = src + (in_i * k_fields);
    *iter_x = point[0];
    *iter_y = point[1];
    *iter_z = point[2];
    *iter_i = point[3];
    *iter_r = point[4];
    *iter_t = point[5] + scan_time_sec;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_i;
    ++iter_r;
    ++iter_t;
  }

  return msg;
}

std::shared_ptr<sensor_msgs::msg::CompressedImage> load_compressed_image(
  const std::string & image_path, const std::string & compression_format, int jpeg_quality)
{
  auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();

  if (compression_format == "png") {
    std::ifstream file(image_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
      return nullptr;
    }

    const std::streamsize size = file.tellg();
    if (size <= 0) {
      return nullptr;
    }

    file.seekg(0, std::ios::beg);
    msg->data.resize(static_cast<size_t>(size));
    if (!file.read(reinterpret_cast<char *>(msg->data.data()), size)) {
      return nullptr;
    }

    msg->format = "png";
    return msg;
  }

  if (compression_format == "jpeg") {
    const cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
      return nullptr;
    }

    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality};
    if (!cv::imencode(".jpg", image, msg->data, params)) {
      return nullptr;
    }

    msg->format = "jpeg";
    return msg;
  }

  return nullptr;
}

}  // namespace boreas
