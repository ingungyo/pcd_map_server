#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "pcd_map_server/pcd_map_server.hpp"

namespace pcd_map_server
{

bool do_remove_ground(sensor_msgs::msg::PointCloud2 & cloud_msg,
                      double /*max_slope_deg*/,
                      rclcpp::Logger logger)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud_msg, *cloud);

  if (cloud->empty()) {
    RCLCPP_WARN(logger, "remove_ground: empty cloud");
    return false;
  }

  // 단순 RANSAC plane 제거 (지면 가정). 실제 프로젝트에 맞게 개선 권장.
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.15);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    RCLCPP_WARN(logger, "remove_ground: no plane found");
    return false;
  }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);  // remove plane
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>());
  extract.filter(*cloud_no_ground);

  pcl::toROSMsg(*cloud_no_ground, cloud_msg);
  cloud_msg.header.frame_id = cloud_msg.header.frame_id;  // keep
  return true;
}

}  // namespace pcd_map_server