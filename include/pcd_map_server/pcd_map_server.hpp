#pragma once
#ifndef PCD_MAP_SERVER__PCD_MAP_SERVER_HPP_
#define PCD_MAP_SERVER__PCD_MAP_SERVER_HPP_

#include <string>
#include <optional>
#include <chrono>
#include <filesystem>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcd_map_server
{

class PcdMapServer : public rclcpp::Node
{
public:
  explicit PcdMapServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // setup
  void set_param();
  void set_subscription();
  void set_publishers();
  void set_service();

  void load_pcd_to_msg();
  void publish_map_once();
  void check_subscribers();

  void on_publish_pcd_map(std_srvs::srv::Trigger::Request::ConstSharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr);

  void on_save_pcd_map(std_srvs::srv::Trigger::Request::ConstSharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr);

  std::optional<sensor_msgs::msg::PointCloud2>
  grab_point_cloud_once(const std::string & topic,
    std::chrono::milliseconds timeout, bool transient_local);

  bool save_cloud_to_pcd(const sensor_msgs::msg::PointCloud2 & cloud,
    std::string & saved_path) const;

  // utils
  static std::string timestamped_name(const std::string & base, const std::string & ext);
  static std::string normalize_pcd_path(const std::string &dir, const std::string &name);

private:
  // params
  std::string pcd_map_topic_;
  std::string map_frame_id_;
  std::string pcd_dir_;
  std::string pcd_file_name_;
  bool publish_on_start_{true};

  std::string input_cloud_topic_;
  std::string output_dir_;
  std::string output_file_name_;
  int  snapshot_timeout_sec_{3};
  bool snapshot_transient_local_{false};
  

  // derived
  std::string pcd_file_path_;

  // pubs/services/timer
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_srv_;
  rclcpp::TimerBase::SharedPtr sub_check_timer_;
  rclcpp::CallbackGroup::SharedPtr snapshot_cbg_; // one-shot 전용 콜백 그룹

  // state
  std::optional<sensor_msgs::msg::PointCloud2> pcd_loaded_msg_;
  size_t last_subs_{0};
};

} // namespace pcd_map_server

#endif // PCD_MAP_SERVER__PCD_MAP_SERVER_HPP_
