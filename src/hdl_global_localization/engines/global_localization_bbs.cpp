#include <hdl_global_localization/engines/global_localization_bbs.hpp>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <hdl_global_localization/bbs/bbs_localization.hpp>
#include <hdl_global_localization/bbs/occupancy_gridmap.hpp>
#include <vector>
#include <iostream>
#include <omp.h>
namespace hdl_global_localization {

GlobalLocalizationBBS::GlobalLocalizationBBS(ros::NodeHandle& private_nh) : private_nh(private_nh) {
  gridmap_pub = private_nh.advertise<nav_msgs::OccupancyGrid>("bbs/gridmap", 1, true);
  map_slice_pub = private_nh.advertise<sensor_msgs::PointCloud2>("bbs/map_slice", 1, true);
  scan_slice_pub = private_nh.advertise<sensor_msgs::PointCloud2>("bbs/scan_slice", 1, false);
}

GlobalLocalizationBBS ::~GlobalLocalizationBBS() {}

void GlobalLocalizationBBS::set_global_map(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud) {
  BBSParams params;
  params.max_range = private_nh.param<double>("bbs/max_range", 15.0);
  params.min_tx = private_nh.param<double>("bbs/min_tx", -10.0);
  params.max_tx = private_nh.param<double>("bbs/max_tx", 10.0);
  params.min_ty = private_nh.param<double>("bbs/min_ty", -10.0);
  params.max_ty = private_nh.param<double>("bbs/max_ty", 10.0);
  params.min_theta = private_nh.param<double>("bbs/min_theta", -3.15);
  params.max_theta = private_nh.param<double>("bbs/max_theta", 3.15);
  bbs.reset(new BBSLocalization(params));

  double map_min_z = private_nh.param<double>("bbs/map_min_z", 2.0);
  double map_max_z = private_nh.param<double>("bbs/map_max_z", 2.4);
  auto map_2d = slice(*cloud);
  ROS_INFO_STREAM("Set Map " << map_2d.size() << " points");

  if (map_2d.size() < 128) {
    ROS_WARN_STREAM("Num points in the sliced map is too small!!");
    ROS_WARN_STREAM("Change the slice range parameters!!");
  }

  int map_width = private_nh.param<int>("bbs/map_width", 1024);
  int map_height = private_nh.param<int>("bbs/map_height", 1024);
  double map_resolution = private_nh.param<double>("bbs/map_resolution", 0.5);
  int map_pyramid_level = private_nh.param<int>("bbs/map_pyramid_level", 6);
  int max_points_per_cell = private_nh.param<int>("bbs/max_points_per_cell", 5);
  bbs->set_map(map_2d, map_resolution, map_width, map_height, map_pyramid_level, max_points_per_cell);

  auto map_3d = unslice(map_2d);
  map_3d->header.frame_id = "map";
  map_slice_pub.publish(map_3d);
  gridmap_pub.publish(bbs->gridmap()->to_rosmsg());
}

GlobalLocalizationResults GlobalLocalizationBBS::query(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, int max_num_candidates) {
  double scan_min_z = private_nh.param<double>("bbs/scan_min_z", -0.2);
  double scan_max_z = private_nh.param<double>("bbs/scan_max_z", 0.2);
  auto scan_2d = slice_scan(*cloud, scan_min_z, scan_max_z); // 라이다를 파라미터로 자를때 
  // auto scan_2d = slice(*cloud); // 라이다를 그리드로 자를때 
  std::vector<GlobalLocalizationResult::Ptr> results;

  ROS_INFO_STREAM("Query " << scan_2d.size() << " points");
  if (scan_2d.size() < 32) {
    ROS_WARN_STREAM("Num points in the sliced scan is too small!!");
    ROS_WARN_STREAM("Change the slice range parameters!!");
    return GlobalLocalizationResults(results);
  }

  double best_score = 0.0;
  auto trans_2d = bbs->localize(scan_2d, 0.0, &best_score);
  if (trans_2d == boost::none) {
    return GlobalLocalizationResults(results);
  }

  if (scan_slice_pub.getNumSubscribers()) {
    auto scan_3d = unslice(scan_2d);
    scan_3d->header = cloud->header;
    scan_slice_pub.publish(scan_3d);
  }

  Eigen::Isometry3f trans_3d = Eigen::Isometry3f::Identity();
  trans_3d.linear().block<2, 2>(0, 0) = trans_2d->linear();
  trans_3d.translation().head<2>() = trans_2d->translation();

  results.resize(1);
  results[0].reset(new GlobalLocalizationResult(best_score, best_score, trans_3d));

  return GlobalLocalizationResults(results);
}

GlobalLocalizationBBS::Points2D GlobalLocalizationBBS::slice_scan(const pcl::PointCloud<pcl::PointXYZ>& cloud, double min_z, double max_z) const {
  Points2D points_2d;
  points_2d.reserve(cloud.size());
  for (int i = 0; i < cloud.size(); i++) {
    if (min_z < cloud.at(i).z && cloud.at(i).z < max_z) {
      points_2d.push_back(cloud.at(i).getVector3fMap().head<2>());
    }
  }
  return points_2d;
}

GlobalLocalizationBBS::Points2D GlobalLocalizationBBS::slice(const pcl::PointCloud<pcl::PointXYZ>& cloud) const {
  Points2D points_2d;


  float min_x = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  for (const auto& point : cloud) {
    if (point.x < min_x) min_x = point.x;
    if (point.x > max_x) max_x = point.x;
  }

  // 2. pcd의 최소 y값부터 최대 y값의 범위내에서 반복
  float min_y = std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  for (const auto& point : cloud) {
    if (point.y < min_y) min_y = point.y;
    if (point.y > max_y) max_y = point.y;
  }

  float resolution = 10.0f;

  for (float x = min_x; x <= max_x; x +=resolution ) {
    for (float y = min_y; y <= max_y; y += resolution) {
      float min_z_in_range = std::numeric_limits<float>::max();
      for (const auto& point : cloud) {
        if (point.x >= x && point.x <= x + resolution && point.y >= y && point.y <= y + resolution) {
          min_z_in_range = std::min(min_z_in_range, point.z);
          
        }
      }

      if (min_z_in_range > 40)
        continue;
      // std::cout << x<< "와 " << x+resolution << ",  " << y<< "와 " << y+resolution << ",사이에서의 최소값 : " << min_z_in_range << std::endl; 

      for (const auto& point : cloud) {
        if (point.x >= x && point.x <= x + resolution && point.y >= y && point.y <= y + resolution &&
            point.z >= min_z_in_range +3.0 && point.z <= min_z_in_range + 4.5) {
              // point.z >= min_z_in_range +3.5 && point.z <= min_z_in_range + 4.0){
          points_2d.push_back(point.getVector3fMap().head<2>());
          
        }
      }
    }
  }

  std::cout << "min_x :" << min_x << "  max_x :" << max_x << std::endl;
  std::cout << "min_y :" << min_y << "  max_y :" << max_y << std::endl;
  std::cout << "the number of cloud for global localization point is :" << points_2d.size() << std::endl;
  std::cout << "*****global map slice done*****" <<std::endl;
  return points_2d;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GlobalLocalizationBBS::unslice(const Points2D& points) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(points.size());
  for (int i = 0; i < points.size(); i++) {
    cloud->at(i).getVector3fMap().head<2>() = points[i];
    cloud->at(i).z = 0.0f;
  }

  return cloud;
}
}  // namespace hdl_global_localization