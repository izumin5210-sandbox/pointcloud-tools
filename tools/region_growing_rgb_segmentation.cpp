//
// Created by Masayuki IZUMI on 5/14/16.
// ref: http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php
//
#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

int
main (int argc, char** argv)
{
  auto tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBA>>(new pcl::search::KdTree<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

  auto pcd_file = argv[1];
  if (pcl::io::loadPCDFile(pcd_file, *cloud) == -1) {
    std::cout << "Cloud reading failed." << std::endl;
    return -1;
  }

  pcl::IndicesPtr indices(new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGBA> reg;
  reg.setInputCloud(cloud);
  reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(10);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(600);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  std::cout << *cloud << std::endl;
  auto colored_cloud = reg.getColoredCloudRGBA();
  std::cout << *colored_cloud << std::endl;

  pcl::visualization::PCLVisualizer viewer("Cluster viewer");
  viewer.addPointCloud(colored_cloud);

  while (!viewer.wasStopped()) {
    viewer.spinOnce();
    boost::this_thread::sleep(boost::posix_time::microseconds(100));
  }

  return 0;
}

// 6/2 10:20 ~ 恵比寿ガーデンプレイス 32F
// 筆記用具と学生証
// 当日連絡 080-1232-9667