#include "pcd_map_server/pcd_map_server.hpp"

using namespace std::chrono_literals;

namespace fs = std::filesystem;

namespace {

  inline std::string normalize_pcd_path(const std::string &dir, const std::string &name)
  {
    fs::path p = fs::path(dir) / name;
    if (p.extension() != ".pcd") p.replace_extension(".pcd");
    return p.string();
  }
  
} // namespace

namespace pcd_map_server
{
  PcdMapServer::PcdMapServer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("pcd_map_server", options)
  {
    set_param();
    set_subscription();
    set_publishers();
    set_service();

      // 미리 PCD 로드 및 필요 시 퍼블리시
    if (!pcd_file_path_.empty() && fs::exists(pcd_file_path_)) {
      load_pcd_to_msg();
      if (publish_on_start_ && pcd_loaded_msg_) {
        publish_once();
      }
    } else if (!pcd_file_path_.empty()) {
      RCLCPP_WARN(get_logger(), "PCD file does not exist: %s", pcd_file_path_.c_str());
    }
    
    sub_check_timer_ = this->create_wall_timer(1s, [this]() { check_subscribers(); });
    snapshot_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RCLCPP_INFO(get_logger(), "pcd_map_server node initialized. output_dir='%s'",
                output_dir_.c_str());
  }
  
  void PcdMapServer::set_param()
  { 
    pcd_map_topic_ = this->declare_parameter<std::string>("pcd_map_topic", "pcd/map");
    input_cloud_topic_ = this->declare_parameter<std::string>("input_cloud_topic", "scan/points");
    map_frame_id_ = this->declare_parameter<std::string>("map_frame_id", "map");
    pcd_dir_ = this->declare_parameter<std::string>("pcd_dir", "./");
    pcd_file_name_ = this->declare_parameter<std::string>("pcd_file_name", "map");
    output_dir_ = this->declare_parameter<std::string>("output_dir", "./");
    remove_ground_max_slope_deg_ = this->declare_parameter<double>("remove_ground_max_slope_deg", 10.0);
    publish_on_start_ = this->declare_parameter<bool>("publish_on_start", true);
    snapshot_timeout_sec_      = this->declare_parameter<int>("snapshot_timeout_sec", 3);
    snapshot_transient_local_  = this->declare_parameter<bool>("snapshot_transient_local", false);

    pcd_file_path_ = normalize_pcd_path(pcd_dir_, pcd_file_name_);

    try {
      fs::create_directories(output_dir_);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed to create output_dir='%s': %s",
                  output_dir_.c_str(), e.what());
    }
  }
  
  void PcdMapServer::set_subscription()
  {
    auto qos = rclcpp::SensorDataQoS().reliable();  // 명시적 QoS
    // cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud_topic_, qos,
    //   std::bind(&PcdMapServer::cloud_callback, this, std::placeholders::_1));
  }

  void PcdMapServer::set_publishers()
  {
    rclcpp::QoS qos(1);
    qos.transient_local().reliable();
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pcd_map_topic_, qos);
  }
    
  void PcdMapServer::set_service()
  {
    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "save_pcd_map",
      std::bind(&PcdMapServer::on_save_pcd_map, this,
                std::placeholders::_1, std::placeholders::_2));

    remove_ground_srv_ = this->create_service<std_srvs::srv::SetBool>(
      "remove_ground",
      std::bind(&PcdMapServer::on_remove_ground, this,
                std::placeholders::_1, std::placeholders::_2));

    publish_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "publish_pcd_map",
      std::bind(&PcdMapServer::on_publish_pcd_map, this,
                std::placeholders::_1, std::placeholders::_2));

  }

  std::optional<sensor_msgs::msg::PointCloud2>
  PcdMapServer::grab_one_cloud_once(const std::string & topic,
                                    std::chrono::milliseconds timeout,
                                    bool transient_local)
  {
    // QoS 구성
    rclcpp::QoS qos = rclcpp::SensorDataQoS(); // 기본 reliable
    if (transient_local) {
      qos.keep_last(1).reliable().transient_local();
    }

    // 첫 프레임을 "포인터"로 받아 불필요한 복사 방지
    using CloudPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

    std::promise<CloudPtr> prom;
    auto fut = prom.get_future();
    std::atomic_bool done{false};

    // 콜백 그룹 옵션 세팅 (set_callback_group 대신 멤버에 직접 대입)
    rclcpp::SubscriptionOptions opts;
    opts.callback_group = snapshot_cbg_;  // snapshot_cbg_는 MutuallyExclusive로 생성해두세요

    // 임시 구독: 첫 메시지 받으면 promise fulfill
    auto sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, qos,
      [&prom, &done](CloudPtr msg){
        bool expected = false;
        if (done.compare_exchange_strong(expected, true)) {
          prom.set_value(msg);  // 복사 없음
        }
      },
      opts
    );


    // 전용 executor에 "해당 콜백 그룹"만 등록해서 돌림
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_callback_group(snapshot_cbg_, this->get_node_base_interface());

    auto ret = exec.spin_until_future_complete(fut, timeout);
    exec.cancel();
    sub.reset();  // 구독 해제

    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      return std::nullopt; // TIMEOUT / INTERRUPTED
    }

    // 실제 값은 복사 1회만 일어나도록 반환 시점에 생성
    // (필요 없으면 여기서도 복사 없이 바로 저장 루틴으로 넘겨도 됩니다)
    return *(fut.get()); // CloudPtr -> 객체 복사 1회
  }

  bool PcdMapServer::save_cloud_to_pcd(const sensor_msgs::msg::PointCloud2 & cloud,
                                     std::string & saved_path) const
  {
    try { fs::create_directories(output_dir_); } catch (...) {}

    const std::string filename = timestamped_name("map", "pcd");
    saved_path = (fs::path(output_dir_) / filename).string();

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);

    pcl::PCDWriter writer;
    // 바이너리 (압축 없이)
    int ret = writer.writeBinary(saved_path, pcl_pc2);

    // 압축으로 저장하려면 ↓ 사용
    // int ret = writer.writeBinaryCompressed(saved_path, pcl_pc2);

    if (ret != 0) {
      RCLCPP_ERROR(get_logger(), "PCD save failed (%d): %s", ret, saved_path.c_str());
      return false;
    }
    return true;
  }

  void PcdMapServer::on_save_pcd_map(std_srvs::srv::Trigger::Request::ConstSharedPtr,
                                      std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    const bool ok = this->save_pcd_map(output_dir_);
    res->success = ok;
    res->message = ok
      ? "Saved map.pcd, map.pgm, map.yaml"
      : "Save failed (no cloud or I/O error)";
  }

  void PcdMapServer::on_remove_ground(std_srvs::srv::SetBool::Request::ConstSharedPtr request,
                                        std_srvs::srv::SetBool::Response::SharedPtr response)
  {
    if (!request->data) {
    response->success = true;
    response->message = "No-op";
    return;
    }

    const bool ok = this->remove_ground(remove_ground_max_slope_deg_);
    response->success = ok;
    response->message = ok ? "Ground removed" : "Ground removal failed (no cloud)";
  }

  void PcdMapServer::on_publish_pcd_map(const std_srvs::srv::Trigger::Request::ConstSharedPtr,
                                          std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    if (!pcd_loaded_msg_) {
    load_pcd_to_msg();
    }
    if (pcd_loaded_msg_) {
      publish_once();
      res->success = true;
      res->message = "PCD map published";
    } else {
      res->success = false;
      res->message = "PCD not loaded (check pcd_file param)";
    }
  }

  void PcdMapServer::load_pcd_to_msg()
  {
    if (pcd_file_path_.empty()) {
      RCLCPP_WARN(get_logger(), "Param 'pcd_file' is empty; skip loading.");
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

  void PcdMapServer::publish_once()
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
      // 새 구독자 등장 → 한 번 더 전송
      publish_once();
    }
    last_subs_ = subs;
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
            
            
  bool PcdMapServer::save_pcd_map(const std::string & out_dir) const
  {
  //   auto cloud_opt = latest_cloud();
  //   if (!cloud_opt) {
  //     return false;
  //   }

  //   try {
  //     fs::create_directories(out_dir);
  //   } catch (const std::exception & e) {
  //     RCLCPP_ERROR(get_logger(), "Failed to create directory: %s", e.what());
  //     return false;
  //   }

  //   const auto pcd_path  = (fs::path(out_dir) / "map.pcd").string();
  //   const auto pgm_path  = (fs::path(out_dir) / "map.pgm").string();
  //   const auto yaml_path = (fs::path(out_dir) / "map.yaml").string();

  // // 실제 저장 로직은 별도 cpp에 구현(여기선 선언만)
  //   extern bool do_save_pcd_pgm_yaml(const sensor_msgs::msg::PointCloud2 & cloud,
  //   const std::string & pcd_path,
  //   const std::string & pgm_path,
  //   const std::string & yaml_path,
  //   const std::string & map_frame,
  //   rclcpp::Logger logger);
    
  //   bool ok = do_save_pcd_pgm_yaml(*cloud_opt, pcd_path, pgm_path, yaml_path, map_frame_id_, get_logger());
  //   if (ok) {
  //     RCLCPP_INFO(get_logger(), "Saved: %s, %s, %s", pcd_path.c_str(), pgm_path.c_str(), yaml_path.c_str());
  //   }
  //   return ok;
  }

  bool PcdMapServer::remove_ground(double max_slope_deg)
  {
    // auto cloud_opt = latest_cloud();
    // if (!cloud_opt) {
    //   return false;
    // }
    
    // extern bool do_remove_ground(sensor_msgs::msg::PointCloud2 & cloud,
    //   double max_slope_deg,
    //   rclcpp::Logger logger);
      
    //   auto cloud_copy = *cloud_opt;  // copy-in-place
    //   bool ok = do_remove_ground(cloud_copy, max_slope_deg, get_logger());
    //   if (ok) {
    //     // last_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud_copy);
    //     RCLCPP_INFO(get_logger(), "Ground removal done. Updated last_cloud.");
    //   }
    //   return ok;
  }
  
}  // namespace pcd_map_server
              