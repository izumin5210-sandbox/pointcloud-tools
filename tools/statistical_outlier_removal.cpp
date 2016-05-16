//
// Created by Masayuki IZUMI on 5/14/16.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "pctools/utils.hpp"

using namespace pctools;

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);

  std::string pcd_file = argv[1];
  pcl::io::loadPCDFile(pcd_file, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::visualization::PCLVisualizer viewer("viewer");
  std::vector<int> viewports;
  splitVisualizer(viewer, 2, 1, viewports);

  viewer.addPointCloud(cloud, "input", viewports[0]);
  viewer.addPointCloud(cloud_filtered, "filtered", viewports[1]);

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }

  return (0);
}