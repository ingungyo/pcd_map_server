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
  use_pcd_file_ = this->declare_parameter<bool>("save.use_pcd_file", false);
  input_cloud_topic_ = this->declare_parameter<std::string>("save.input_cloud_topic", "Laser_map");
  output_dir_ = this->declare_parameter<std::string>("save.output_dir", "./");
  output_file_name_ = this->declare_parameter<std::string>("save.output_file_name", "map");
  snapshot_timeout_sec_ = this->declare_parameter<int>("save.snapshot_timeout_sec", 3);
  snapshot_transient_local_ = this->declare_parameter<bool>("save.snapshot_transient_local", false);

  // save gird params
  proj_z_min_ = this->declare_parameter<double>("save_grid.proj_z_min", -0.0);
  proj_z_max_ = this->declare_parameter<double>("save_grid.proj_z_max",  1.5);
  proj_resolution_ = this->declare_parameter<double>("save_grid.proj_resolution", 0.05);
  proj_padding_m_ = this->declare_parameter<double>("save_grid.proj_padding_m",  1.0);
  proj_dilate_radius_px_ = this->declare_parameter<int>("save_grid.proj_dilate_radius_px", 1);
  ground_z_min_ = this->declare_parameter<double>("save_grid.ground_z_min", 0.2);
  ground_z_max_ = this->declare_parameter<double>("save_grid.ground_z_max", 1.5);
  obstacle_z_thresh_ = this->declare_parameter<double>("save_grid.obstacle_z_thresh", 0.2);
  use_intensity_ = this->declare_parameter<bool>("save_grid.use_intensity", false);
  intensity_thresh_ = this->declare_parameter<double>("save_grid.intensity_thresh", 50.0);
  yaml_occupied_thresh_ = this->declare_parameter<double>("save_grid.yaml_occupied_thresh", 0.65);
  yaml_free_thresh_ = this->declare_parameter<double>("save_grid.yaml_free_thresh", 0.25);
  yaml_mode_ = this->declare_parameter<std::string>("save_grid.yaml_mode", "trinary");
  yaml_negate_ = this->declare_parameter<int>("save_grid.yaml_negate", 0);
  
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

  sensor_msgs::msg::PointCloud2 pointcloud_data;

  if (use_pcd_file_) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path_, cloud) != 0) {
      res->success = false;
      res->message = "Failed to load existing PCD file: " + pcd_file_path_;
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }
    pcl::toROSMsg(cloud, pointcloud_data);
    pointcloud_data.header.frame_id = map_frame_id_;

    RCLCPP_INFO(get_logger(), "Loaded existing PCD for projection: %s", pcd_file_path_.c_str());
  }

  else{
    const auto timeout = std::chrono::seconds(snapshot_timeout_sec_);
    auto cloud_opt = grab_pointcloud_once(input_cloud_topic_, timeout, snapshot_transient_local_);

    if (!cloud_opt) {
      res->success = false;
      res->message = "No cloud received within timeout";
      RCLCPP_WARN(get_logger(), "One-shot capture timed out after %d sec", snapshot_timeout_sec_);
      return;
    }
    
    pointcloud_data = *cloud_opt;

    std::string pcd_path;

    if (!save_pointcloud_to_pcd(*cloud_opt, pcd_path)) {
      res->success = false;
      res->message = "PCD save failed";
      return;
    }
    RCLCPP_INFO(get_logger(), "Saved new PCD: %s", pcd_path.c_str());
  }
  

  const std::string stem = timestamped_name(output_file_name_, "tmp");
  const std::string clean_stem = stem.substr(0, stem.size() - 4);

  std::string pgm_path, yaml_path;

  if (!save_pointcloud_to_pgm(pointcloud_data, output_dir_, clean_stem, pgm_path, yaml_path)) {
    RCLCPP_WARN(get_logger(), "PCD saved but PGM/YAML creation failed.");
    res->success = use_pcd_file_;
    res->message = "PGM/YAML creation failed.";
    return;
  }

  res->success = true;
  std::ostringstream oss;
  if (use_pcd_file_) {
    oss << "Generated PGM/YAML from existing PCD: " << fs::absolute(pcd_file_path_).string()
        << " | PGM: " << pgm_path
        << " | YAML: " << yaml_path;
  } else {
    oss << "Saved PCD + Generated PGM/YAML"
        << " | PGM: " << pgm_path
        << " | YAML: " << yaml_path;
  }
  res->message = oss.str();

  RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
}

std::optional<sensor_msgs::msg::PointCloud2>
PcdMapServer::grab_pointcloud_once(const std::string & topic,
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

bool PcdMapServer::save_pointcloud_to_pcd(const sensor_msgs::msg::PointCloud2 & cloud,
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

bool PcdMapServer::save_pointcloud_to_pgm(
  const sensor_msgs::msg::PointCloud2& cloud,
  const std::string& out_dir,
  const std::string& stem,
  std::string& pgm_path_out,
  std::string& yaml_path_out) const
{
  // 1) ROS → PCL
  pcl::PointCloud<pcl::PointXYZ> pc;
  pcl::fromROSMsg(cloud, pc);

  // 2) Z 필터링
  std::vector<Eigen::Vector2f> xy;
  xy.reserve(pc.size());
  double min_x=std::numeric_limits<double>::infinity(),
         min_y=std::numeric_limits<double>::infinity(),
         max_x=-std::numeric_limits<double>::infinity(),
         max_y=-std::numeric_limits<double>::infinity();

  for (const auto& p : pc.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.z < proj_z_min_ || p.z > proj_z_max_) continue;
    xy.emplace_back(p.x, p.y);
    min_x = std::min<double>(min_x, p.x);
    min_y = std::min<double>(min_y, p.y);
    max_x = std::max<double>(max_x, p.x);
    max_y = std::max<double>(max_y, p.y);
  }
  if (xy.empty()) {
    RCLCPP_WARN(get_logger(), "Projection found no points in z range [%.3f, %.3f].",
                proj_z_min_, proj_z_max_);
    return false;
  }

  // 3) 격자 크기/원점 계산
  const double pad = std::max(0.0, proj_padding_m_);
  const double res = proj_resolution_ > 0.0 ? proj_resolution_ : 0.05;

  const double min_x_pad = min_x - pad;
  const double max_x_pad = max_x + pad;
  const double min_y_pad = min_y - pad;
  const double max_y_pad = max_y + pad;

  const int width  = static_cast<int>(std::ceil((max_x_pad - min_x_pad) / res));
  const int height = static_cast<int>(static_cast<int>(std::ceil((max_y_pad - min_y_pad) / res)));

  if (width <= 0 || height <= 0) {
    RCLCPP_WARN(get_logger(), "Invalid grid size (w=%d,h=%d).", width, height);
    return false;
  }

  // Nav2/map_server 관례: 0=occupied(검정), 254=free(흰색), 205=unknown
  const uint8_t OCC = 0, FREE = 254, UNK = 205;

  std::vector<uint8_t> img(static_cast<size_t>(width) * height, UNK);

  struct CellStat { double min_z=std::numeric_limits<double>::infinity();
                    double max_z=-std::numeric_limits<double>::infinity();
                    bool observed=false; };
  std::vector<CellStat> stat(static_cast<size_t>(width) * height);

  for (const auto& p : pc.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    if (p.z < proj_z_min_ || p.z > proj_z_max_) continue; // 투영용 z 필터가 있다면 유지
  
    const int col = static_cast<int>(std::floor((p.x - min_x_pad) / res));
    const int row = static_cast<int>(std::floor((max_y_pad - p.y) / res));
    if (col < 0 || col >= width || row < 0 || row >= height) continue;
  
    auto& s = stat[static_cast<size_t>(row) * width + col];
    s.observed = true;
    if (p.z < s.min_z) s.min_z = p.z;
    if (p.z > s.max_z) s.max_z = p.z;
  }

  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) {
      auto& s = stat[static_cast<size_t>(r) * width + c];
      auto& px = img[static_cast<size_t>(r) * width + c];
      if (!s.observed) { px = UNK; continue; } // 미관측=UNKNOWN
  
      if (s.max_z >= obstacle_z_thresh_) {
        px = OCC;               // 높은 점이 있으면 장애물
      } else if (s.min_z <= ground_z_max_ && s.min_z >= ground_z_min_) {
        px = FREE;              // 지면 대역을 본 셀
      } else {
        px = UNK;               // 지면도 아니고 높은 장애물도 아님 → 근거 부족
      }
    }
  }

  // int r = std::max(0, proj_dilate_radius_px_);
  const int r = std::max(0, static_cast<int>(std::lround(proj_dilate_radius_px_)));
  if (r > 0) {
    std::vector<uint8_t> tmp = img;
    for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x) {
      if (tmp[static_cast<size_t>(y)*width + x] == OCC) {
        for (int dy=-r; dy<=r; ++dy) for (int dx=-r; dx<=r; ++dx) {
          int nx=x+dx, ny=y+dy; if (nx<0||nx>=width||ny<0||ny>=height) continue;
          img[static_cast<size_t>(ny)*width + nx] = OCC;
        }
      }
    }
  }



  // 4) 라스터라이즈
  for (const auto& v : xy) {
    // y축 위쪽이 이미지 0행이 되도록 row는 max_y 기준으로 계산(상하반전 방지)
    const int col = static_cast<int>(std::floor((v.x() - min_x_pad) / res));
    const int row = static_cast<int>(std::floor((max_y_pad - v.y()) / res)); // <— 주의
    if (col < 0 || col >= width || row < 0 || row >= height) continue;
    img[row * width + col] = OCC; // 점 하나라도 있으면 점유로 표시
  }

  // // 5) (옵션) 팽창: 얇은 라인 메꿔 탐색성 보강
  // const int r = std::max(0, proj_dilate_radius_px_);
  if (r > 0) {
    std::vector<uint8_t> tmp = img;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (tmp[y*width + x] == OCC) {
          for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
              int nx = x + dx, ny = y + dy;
              if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
              img[ny*width + nx] = OCC;
            }
          }
        }
      }
    }
  }

  // 6) 파일 경로
  try { fs::create_directories(out_dir); } catch (...) {}
  const std::string pgm_name  = stem + ".pgm";
  const std::string yaml_name = stem + ".yaml";
  const fs::path pgm_path  = fs::path(out_dir) / pgm_name;
  const fs::path yaml_path = fs::path(out_dir) / yaml_name;

  // 7) PGM 쓰기 (P5, 8-bit)
  {
    std::ofstream ofs(pgm_path, std::ios::binary);
    if (!ofs) {
      RCLCPP_ERROR(get_logger(), "Failed to open PGM for write: %s", pgm_path.string().c_str());
      return false;
    }
    ofs << "P5\n" << width << " " << height << "\n255\n";
    ofs.write(reinterpret_cast<const char*>(img.data()), static_cast<std::streamsize>(img.size()));
  }

  // 8) YAML 쓰기 (Nav2 map_server 포맷)
  // origin 은 격자 (0,0) 픽셀이 나타내는 월드 좌표의 좌하단이 되도록 설정
  // 우리가 row를 max_y 기준으로 계산했으므로, origin = (min_x_pad, min_y_pad, 0)
  {
    std::ofstream yfs(yaml_path);
    if (!yfs) {
      RCLCPP_ERROR(get_logger(), "Failed to open YAML for write: %s", yaml_path.string().c_str());
      return false;
    }
    yfs << "image: " << pgm_name << "\n";
    yfs << "resolution: " << std::fixed << std::setprecision(6) << res << "\n";
    yfs << "origin: [" << std::fixed << std::setprecision(6) << min_x_pad << ", "
        << min_y_pad << ", 0.0]\n";
    yfs << "mode: " << yaml_mode_ << "\n";
    yfs << "negate: " << yaml_negate_ << "\n";
    yfs << "occupied_thresh: " << yaml_occupied_thresh_ << "\n";
    yfs << "free_thresh: " << yaml_free_thresh_ << "\n";
  }

  pgm_path_out  = fs::absolute(pgm_path).string();
  yaml_path_out = fs::absolute(yaml_path).string();
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
