#include <string>
#include <fstream>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "pcd_map_server/pcd_map_server.hpp"

namespace pcd_map_server
{
bool do_save_pcd_pgm_yaml(const sensor_msgs::msg::PointCloud2 & cloud_msg,
                          const std::string & pcd_path,
                          const std::string & pgm_path,
                          const std::string & yaml_path,
                          const std::string & map_frame,
                          rclcpp::Logger logger)
{
  // --- Save PCD ---
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);
  if (pcl::io::savePCDFileBinary(pcd_path, cloud) != 0) {
    RCLCPP_ERROR(logger, "Failed to save PCD: %s", pcd_path.c_str());
    return false;
  }

  // --- Save PGM & YAML ---
  // 실제 occupancy grid 생성/투영은 프로젝트별 요구가 다름.
  // 여기서는 최소한의 플레이스홀더 파일만 생성 (TODO로 명시)
  // ※ 실제 구현 시 PointCloud -> 2D grid projection + scale/resolution 처리 필요.
  {
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm) {
      RCLCPP_ERROR(logger, "Failed to create PGM: %s", pgm_path.c_str());
      return false;
    }
    // PGM 헤더(placeholder): P2, width height max_value
    pgm << "P2\n1 1\n255\n255\n";
  }
  {
    std::ofstream yaml(yaml_path);
    if (!yaml) {
      RCLCPP_ERROR(logger, "Failed to create YAML: %s", yaml_path.c_str());
      return false;
    }
    yaml << "image: " << pgm_path.substr(pgm_path.find_last_of("/\\") + 1) << "\n";
    yaml << "resolution: 0.05\n";
    yaml << "origin: [0.0, 0.0, 0.0]\n";
    yaml << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  }

  return true;
}

}  // namespace pcd_map_server