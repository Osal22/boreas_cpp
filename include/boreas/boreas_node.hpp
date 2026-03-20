#ifndef BOREADS_NODE_HPP_
#define BOREADS_NODE_HPP_

#include <boreas/boreas.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/static_transform_broadcaster.h>

#include <mutex>

namespace boreas
{
typedef long long int l_int;
class BoreasNode : public rclcpp::Node
{
public:
  BoreasNode();
  ~BoreasNode() = default;

private:
  void clean_string(std::string & in);
  void remove_slash_and_bin(std::string & in);
  long long int path_to_int(std::string & in);
  long long int path_to_int_cam(std::string & in);
  sensor_msgs::msg::Image::SharedPtr read_image(std::string & image_path);

  sensor_msgs::msg::PointCloud2 eigen_to_pointcloud(const Eigen::MatrixXd & pc);
  std::vector<std::vector<double>> read_matrix_from_file(const std::string & filename);
  void publish_transform(const std::vector<std::vector<double>> & matrix);
  void publish_transform_op(std::string & path);
  bool load_camera_info();
  void sync_time_stamps(
    const std::vector<std::pair<long long int, std::string>> & camera_data,
    const std::vector<std::pair<long long int, std::string>> & lidar_dat);

  rclcpp::Time id_to_stamp(long long int ros_time);

  void function1();
  void function2();
  void function3();

  template <typename MsgT>
  rclcpp::SerializedMessage serialize_message(const MsgT & msg);

  std::string data_path_;
  std::string lidar_data_path_;
  std::string camera_data_path_;
  std::string camera_to_lidar_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_;
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;
  rclcpp::TimerBase::SharedPtr timer3_;

  rclcpp::CallbackGroup::SharedPtr callback_group1_;
  rclcpp::CallbackGroup::SharedPtr callback_group2_;
  rclcpp::CallbackGroup::SharedPtr callback_group3_;

  std::unordered_map<long long int, std::string> camera_sorted_;
  std::vector<std::pair<long long int, std::string>> camera_sorted_vec_;
  std::vector<std::pair<long long int, sensor_msgs::msg::Image>> camera_msg_sorted_vec_;

  std::unordered_map<long long int, std::string> lidar_sorted_;
  std::vector<std::pair<long long int, std::string>> lidar_sorted_vec_;
  std::vector<std::pair<long long int, sensor_msgs::msg::PointCloud2>> lidar_msg_sorted_vec_;

  std::vector<std::pair<long long int, long long int>> syn_vec_;

  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  std::mutex mtx;  // Mutex for synchronization
  bool lidar_data_ready_, camera_data_ready_, done_;
  long long int init_sec;
};
}  // namespace boreas
#endif  // BOREADS_NODE_HPP_
