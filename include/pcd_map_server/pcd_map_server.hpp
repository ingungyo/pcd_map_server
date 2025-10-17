#ifndef PCD_MAP_SERVER__PCD_MAP_SERVER_HPP_
#define PCD_MAP_SERVER__PCD_MAP_SERVER_HPP_

#include <string>
#include <memory>
#include <optional>
#include <chrono>
#include <filesystem>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pcd_map_server
{
  
  class PcdMapServer : public rclcpp::Node
  {
    public:
    explicit PcdMapServer(const rclcpp::NodeOptions & options);

    private:
    void set_param();
    void set_subscription();
    void set_publishers();
    void set_service();
    
    bool save_pcd_map(const std::string & out_dir) const;
    bool remove_ground(double max_slope_deg);
    void on_save_pcd_map(std_srvs::srv::Trigger::Request::ConstSharedPtr request,
                          std_srvs::srv::Trigger::Response::SharedPtr response);

    std::optional<sensor_msgs::msg::PointCloud2>
    grab_one_cloud_once(const std::string & topic,
                        std::chrono::milliseconds timeout,
                        bool transient_local);
    
    bool save_cloud_to_pcd(const sensor_msgs::msg::PointCloud2 & cloud,
                            std::string & saved_path) const;

    static std::string timestamped_name(const std::string & base, const std::string & ext);



    void on_remove_ground(std_srvs::srv::SetBool::Request::ConstSharedPtr request,
                          std_srvs::srv::SetBool::Response::SharedPtr response);
    void on_publish_pcd_map(std_srvs::srv::Trigger::Request::ConstSharedPtr request,
                          std_srvs::srv::Trigger::Response::SharedPtr response);

    
    
    void load_pcd_to_msg();
    void publish_once();
    void check_subscribers();

    
    // std::optional<sensor_msgs::msg::PointCloud2> latest_cloud() const;
    
    // Callbacks
    void cloud_callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

    // Members
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr remove_ground_srv_;
    rclcpp::TimerBase::SharedPtr sub_check_timer_;

    // State
    mutable std::mutex cloud_mutex_;

    std::optional<sensor_msgs::msg::PointCloud2> pcd_loaded_msg_;
    size_t last_subs_{0};

    // Params
    std::string pcd_map_topic_;     // publish topic
    std::string input_cloud_topic_;  // subscribe topic (NEW)
    std::string map_frame_id_;
    std::string pcd_dir_;
    std::string pcd_file_name_;
    std::string output_dir_;

    double remove_ground_max_slope_deg_{10.0};
    bool publish_on_start_{true};

    double snapshot_timeout_sec_{3};
    bool snapshot_transient_local_{false};

    rclcpp::CallbackGroup::SharedPtr snapshot_cbg_;

    // Derived
    std::string pcd_file_path_; // = pcd_dir_/pcd_file_name_.pcd
  };
}  // namespace pcd_map_server

#endif  // PCD_MAP_SERVER__PCD_MAP_SERVER_HPP_
