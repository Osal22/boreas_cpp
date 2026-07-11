#ifndef BOREAS_GROUND_TRUTH_HPP_
#define BOREAS_GROUND_TRUTH_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace boreas
{

struct BoreasPose
{
  long long timestamp_us{0};
  double easting{0.0};
  double northing{0.0};
  double altitude{0.0};
  double vel_east{0.0};
  double vel_north{0.0};
  double vel_up{0.0};
  double roll{0.0};
  double pitch{0.0};
  double heading{0.0};
  double angvel_z{0.0};
  double angvel_y{0.0};
  double angvel_x{0.0};
};

struct GnssSample
{
  long long timestamp_us{0};
  double gps_time_sec{0.0};
  double easting{0.0};
  double northing{0.0};
  double altitude{0.0};
  double latitude_rad{0.0};
  double longitude_rad{0.0};
};

inline Eigen::Matrix3d yaw_pitch_roll_to_rotation(double yaw, double pitch, double roll)
{
  const Eigen::Matrix3d rot_yaw =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Matrix3d rot_pitch =
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Matrix3d rot_roll =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
  return rot_roll * rot_pitch * rot_yaw;
}

inline Eigen::Isometry3d pose_to_isometry(const BoreasPose & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = yaw_pitch_roll_to_rotation(pose.heading, pose.pitch, pose.roll);
  transform.translation() = Eigen::Vector3d(pose.easting, pose.northing, pose.altitude);
  return transform;
}

inline Eigen::Isometry3d matrix4_to_isometry(const std::vector<std::vector<double>> & matrix)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transform.linear()(i, j) = matrix[i][j];
    }
    transform.translation()(i) = matrix[i][3];
  }
  return transform;
}

inline Eigen::Isometry3d yaw_only_isometry(double yaw_deg)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const double yaw_rad = yaw_deg * M_PI / 180.0;
  transform.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return transform;
}

inline Eigen::Isometry3d params_to_isometry(
  const std::vector<double> & translation, const std::vector<double> & rotation_xyzw)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  const Eigen::Quaterniond quat(
    rotation_xyzw[3], rotation_xyzw[0], rotation_xyzw[1], rotation_xyzw[2]);
  transform.linear() = quat.normalized().toRotationMatrix();
  return transform;
}

inline geometry_msgs::msg::TransformStamped isometry_to_transform(
  const Eigen::Isometry3d & isometry, const std::string & parent_frame,
  const std::string & child_frame)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = child_frame;

  const Eigen::Quaterniond quat(isometry.rotation());
  transform.transform.translation.x = isometry.translation().x();
  transform.transform.translation.y = isometry.translation().y();
  transform.transform.translation.z = isometry.translation().z();
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();

  return transform;
}

inline nav_msgs::msg::Odometry pose_to_odometry(
  const BoreasPose & pose, const std::string & parent_frame, const std::string & child_frame)
{
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = parent_frame;
  odom.child_frame_id = child_frame;

  odom.pose.pose.position.x = pose.easting;
  odom.pose.pose.position.y = pose.northing;
  odom.pose.pose.position.z = pose.altitude;

  const Eigen::Quaterniond quat(yaw_pitch_roll_to_rotation(pose.heading, pose.pitch, pose.roll));
  odom.pose.pose.orientation.x = quat.x();
  odom.pose.pose.orientation.y = quat.y();
  odom.pose.pose.orientation.z = quat.z();
  odom.pose.pose.orientation.w = quat.w();

  odom.twist.twist.linear.x = pose.vel_east;
  odom.twist.twist.linear.y = pose.vel_north;
  odom.twist.twist.linear.z = pose.vel_up;
  odom.twist.twist.angular.x = pose.angvel_x;
  odom.twist.twist.angular.y = pose.angvel_y;
  odom.twist.twist.angular.z = pose.angvel_z;

  return odom;
}

inline sensor_msgs::msg::NavSatFix gnss_to_nav_sat_fix(const GnssSample & sample)
{
  sensor_msgs::msg::NavSatFix fix;
  fix.latitude = sample.latitude_rad * 180.0 / M_PI;
  fix.longitude = sample.longitude_rad * 180.0 / M_PI;
  fix.altitude = sample.altitude;
  fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
  fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix.position_covariance[0] = 0.05 * 0.05;
  fix.position_covariance[4] = 0.05 * 0.05;
  fix.position_covariance[8] = 0.10 * 0.10;
  return fix;
}

inline bool load_pose_csv(
  const std::string & path, std::unordered_map<long long, BoreasPose> & poses)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;
    while (std::getline(ss, token, ',')) {
      values.push_back(std::stod(token));
    }
    if (values.size() < 12) {
      continue;
    }

    BoreasPose pose;
    pose.timestamp_us = static_cast<long long>(values[0]);
    pose.easting = values[1];
    pose.northing = values[2];
    pose.altitude = values[3];
    pose.vel_east = values[4];
    pose.vel_north = values[5];
    pose.vel_up = values[6];
    pose.roll = values[7];
    pose.pitch = values[8];
    pose.heading = values[9];
    pose.angvel_z = values[10];
    pose.angvel_y = values[11];
    pose.angvel_x = values[12];
    poses.emplace(pose.timestamp_us, pose);
  }

  return !poses.empty();
}

inline bool load_gnss_csv(const std::string & path, std::vector<GnssSample> & samples)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream ss(line);
    std::string token;
    std::vector<double> values;
    while (std::getline(ss, token, ',')) {
      values.push_back(std::stod(token));
    }
    if (values.size() < 18) {
      continue;
    }

    GnssSample sample;
    sample.gps_time_sec = values[0];
    sample.timestamp_us = static_cast<long long>(std::llround(values[0] * 1'000'000.0));
    sample.easting = values[1];
    sample.northing = values[2];
    sample.altitude = values[3];
    sample.latitude_rad = values[16];
    sample.longitude_rad = values[17];
    samples.push_back(sample);
  }

  return !samples.empty();
}

inline const BoreasPose * lookup_pose(
  const std::unordered_map<long long, BoreasPose> & poses, long long timestamp_us)
{
  const auto it = poses.find(timestamp_us);
  if (it == poses.end()) {
    return nullptr;
  }
  return &it->second;
}

}  // namespace boreas

#endif  // BOREAS_GROUND_TRUTH_HPP_
