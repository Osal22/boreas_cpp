#ifndef BOREAS_CALIBRATION_HPP_
#define BOREAS_CALIBRATION_HPP_

#include <Eigen/Dense>

#include <string>
#include <vector>

namespace boreas
{

std::vector<std::vector<double>> read_matrix_from_file(const std::string & filename);

bool load_camera_to_lidar_calibration(
  const std::string & path, Eigen::Isometry3d & camera_to_lidar);

}  // namespace boreas

#endif  // BOREAS_CALIBRATION_HPP_
