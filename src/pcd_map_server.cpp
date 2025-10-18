#include "pcd_map_server/pcd_map_server.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace pcd_map_server
{
// ---------- ctor ----------
PcdMapServer::PcdMapServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("pcd_map_server", options)
{
  set_param();
  set_publishers();
  set_service();

  if (!pcd_file_path_.empty() && fs::exists(pcd_file_path_)) {
    load_pcd_to_msg();
    if (publish_on_start_ && pcd_loaded_msg_) {
      publish_map_once();
    }
  } else if (!pcd_file_path_.empty()) {
    RCLCPP_WARN(get_logger(), "PCD file does not exist: %s", pcd_file_path_.c_str());
  }

  sub_check_timer_ = this->create_wall_timer(1s, [this]() { check_subscribers(); });

  RCLCPP_INFO(get_logger(), "pcd_map_server initialized. output_dir='%s'",
              output_dir_.c_str());
}

void PcdMapServer::set_param()
{
  // publish params
  pcd_map_topic_ = this->declare_parameter<std::string>("publish.pcd_map_topic", "pcd/map");
  map_frame_id_ = this->declare_parameter<std::string>("publish.map_frame_id", "map");
  pcd_dir_ = this->declare_parameter<std::string>("publish.pcd_dir", "./");
  pcd_file_name_ = this->declare_parameter<std::string>("publish.pcd_file_name", "map");
  publish_on_start_ = this->declare_parameter<bool>("pbulish.publish_on_start", true);

  // save params
  input_cloud_topic_ = this->declare_parameter<std::string>("save.input_cloud_topic", "Laser_map");
  output_dir_ = this->declare_parameter<std::string>("save.output_dir", "./");
  output_file_name_ = this->declare_parameter<std::string>("save.output_file_name", "map");
  snapshot_timeout_sec_ = this->declare_parameter<int>("save.snapshot_timeout_sec", 3);
  snapshot_transient_local_ = this->declare_parameter<bool>("save.snapshot_transient_local", false);
  
  pcd_file_path_ = normalize_pcd_path(pcd_dir_, pcd_file_name_);

  try { fs::create_directories(output_dir_); }
  catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Failed to create output_dir='%s': %s",
                output_dir_.c_str(), e.what());
  }
}

void PcdMapServer::set_publishers()
{
  rclcpp::QoS qos(1);
  qos.reliable().transient_local();
  map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pcd_map_topic_, qos);
}

void PcdMapServer::set_subscription()
{
  
}

void PcdMapServer::set_service()
{
  save_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "save_pcd_map",
    std::bind(&PcdMapServer::on_save_pcd_map, this,
              std::placeholders::_1, std::placeholders::_2));

  publish_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "publish_pcd_map",
    std::bind(&PcdMapServer::on_publish_pcd_map, this,
              std::placeholders::_1, std::placeholders::_2));
}

// ---------- PCD load/publish ----------
void PcdMapServer::load_pcd_to_msg()
{
  if (pcd_file_path_.empty()) {
    RCLCPP_WARN(get_logger(), "pcd_file is empty; skip loading.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud;
  int ret = pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, cloud);
  if (ret != 0) {
    RCLCPP_ERROR(get_logger(), "Failed to load PCD: '%s' (code %d)", pcd_file_path_.c_str(), ret);
    return;
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = map_frame_id_;
  pcd_loaded_msg_ = msg;

  RCLCPP_INFO(get_logger(), "Loaded PCD: '%s' (points=%zu)",
              pcd_file_path_.c_str(), cloud.points.size());
}

void PcdMapServer::publish_map_once()
{
  if (!pcd_loaded_msg_) {
    RCLCPP_WARN(get_logger(), "No loaded PCD to publish.");
    return;
  }
  auto msg = *pcd_loaded_msg_;
  msg.header.stamp = this->now();
  map_pub_->publish(msg);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "PCD map published");
}

void PcdMapServer::check_subscribers()
{
  size_t subs = map_pub_->get_subscription_count();
  if (subs > last_subs_) {
    publish_map_once();
  }
  last_subs_ = subs;
}

// ---------- services ----------
void PcdMapServer::on_publish_pcd_map(std_srvs::srv::Trigger::Request::ConstSharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  if (!pcd_loaded_msg_) {
    load_pcd_to_msg();
  }
  if (pcd_loaded_msg_) {
    publish_map_once();
    res->success = true;
    res->message = "PCD map published";
  } else {
    res->success = false;
    res->message = "PCD not loaded (check pcd_dir/pcd_file_name)";
  }
}

void PcdMapServer::on_save_pcd_map(std_srvs::srv::Trigger::Request::ConstSharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  const auto timeout = std::chrono::seconds(snapshot_timeout_sec_);
  auto cloud_opt = grab_point_cloud_once(input_cloud_topic_, timeout, snapshot_transient_local_);

  if (!cloud_opt) {
    res->success = false;
    res->message = "No cloud received within timeout";
    RCLCPP_WARN(get_logger(), "One-shot capture timed out after %d sec", snapshot_timeout_sec_);
    return;
  }

  std::string path;
  if (!save_cloud_to_pcd(*cloud_opt, path)) {
    res->success = false;
    res->message = "PCD save failed";
    return;
  }

  res->success = true;
  res->message = "Saved: " + path;
  RCLCPP_INFO(get_logger(), "Saved one-shot PCD: %s", path.c_str());
}

std::optional<sensor_msgs::msg::PointCloud2>
PcdMapServer::grab_point_cloud_once(const std::string & topic,
  std::chrono::milliseconds timeout, bool transient_local)
{
  auto tmp = std::make_shared<rclcpp::Node>(
      "pcd_snapshot_grabber",
      rclcpp::NodeOptions().use_intra_process_comms(true));

  auto cbg = tmp->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  if (transient_local) {
    qos.keep_last(1).reliable().transient_local();
  }

  using CloudPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  std::promise<CloudPtr> prom;
  auto fut = prom.get_future();
  std::atomic_bool done{false};

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = cbg;

  auto sub = tmp->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, qos, [&prom, &done](CloudPtr msg){
      bool expected = false;
      if (done.compare_exchange_strong(expected, true)) {
        prom.set_value(msg);
      }
    },opts);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(tmp);
  auto ret = exec.spin_until_future_complete(fut, timeout);

  exec.cancel();
  sub.reset();
  tmp.reset();

  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    return std::nullopt;
  }

  return *(fut.get());
}

bool PcdMapServer::save_cloud_to_pcd(const sensor_msgs::msg::PointCloud2 & cloud,
  std::string & saved_path) const
{
  try { fs::create_directories(output_dir_); } catch (...) {}

  const std::string filename = timestamped_name(output_file_name_, "pcd");
  saved_path = (fs::path(output_dir_) / filename).string();

  // Pointcloud2 msg to PCLPointCloud2
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud, pcl_pc2);

  pcl::PCDWriter writer;
  int ret = writer.writeBinary(saved_path, pcl_pc2);

  if (ret != 0) {
    RCLCPP_ERROR(get_logger(), "PCD save failed (%d): %s", ret, saved_path.c_str());
    return false;
  }
  return true;
}

// ---------- utils ----------
std::string PcdMapServer::normalize_pcd_path(const std::string &dir, const std::string &name)
{
  fs::path p = fs::path(dir) / name;
  if (p.extension() != ".pcd") p.replace_extension(".pcd");
  return p.string();
}

std::string PcdMapServer::timestamped_name(const std::string & base, const std::string & ext)
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << base << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << "." << ext;
  return oss.str();
}

} // namespace pcd_map_server
