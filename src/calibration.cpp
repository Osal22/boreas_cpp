#include <boreas/calibration.hpp>
#include <boreas/ground_truth.hpp>

#include <fstream>
#include <sstream>

namespace boreas
{

std::vector<std::vector<double>> read_matrix_from_file(const std::string & filename)
{
  std::ifstream file(filename);
  std::vector<std::vector<double>> matrix;
  std::string line;

  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::vector<double> row;
    double value{0.0};
    while (iss >> value) {
      row.push_back(value);
    }
    if (!row.empty()) {
      matrix.push_back(row);
    }
  }
  return matrix;
}

bool load_camera_to_lidar_calibration(
  const std::string & path, Eigen::Isometry3d & camera_to_lidar)
{
  const auto matrix = read_matrix_from_file(path);
  if (matrix.size() != 4 || matrix[0].size() != 4) {
    return false;
  }

  camera_to_lidar = matrix4_to_isometry(matrix);
  return true;
}

}  // namespace boreas
