//
// Created by Masayuki IZUMI on 5/17/16.
//

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pctools/utils.hpp"

int main(int argc, char* argv[]) {
  int noise_filter = 7;
  double resolution = 0.01;

  pcl::console::parse_argument(argc, argv, "-r", resolution);
  pcl::console::parse_argument(argc, argv, "-n", noise_filter);

  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZRGBA>);
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::io::loadPCDFile(argv[argc - 2], *cloud_in1);
  pcl::io::loadPCDFile(argv[argc - 1], *cloud_in2);

  std::vector<int> viewports;
  pcl::visualization::PCLVisualizer viewer("viewer");
  pctools::splitVisualizer(viewer, 3, 1, viewports);

  pctools::extractDifference<pcl::PointXYZRGBA>(cloud_in1, cloud_in2, cloud_out, resolution, noise_filter);
  std::cout << cloud_out->points.size() << std::endl;

  viewer.addPointCloud(cloud_in1, "cloud_in1", viewports[0]);
  viewer.addPointCloud(cloud_in2, "cloud_in2", viewports[1]);
  viewer.addPointCloud(cloud_out, "cloud_out", viewports[2]);

  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }

  return 0;
}
