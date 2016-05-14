//
// Created by Masayuki IZUMI on 5/12/16.
//

#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

#include "pctools/utils.hpp"

namespace pc = pcl::console;
namespace bfs = boost::filesystem;

using namespace pctools;

std::string dir_in;
std::string dir_out;

void print_usage(char *argv[]) {
  std::cout << "Usage: " << argv[0] << " source_dir target_dir\n"
  << std::endl;
}

void parse_opts(int argc, char *argv[]) {
  if (pc::find_switch(argc, argv, "-h") || pc::find_switch(argc, argv, "--help")) {
    print_usage(argv);
    exit(0);
  }

  if (argc < 3) {
    shutdown_with_error("error: source directory and target directory is required.");
  } else if (argc > 3) {
    shutdown_with_error("error: arguments are too much.");
  }
  dir_in = argv[1];
  dir_out = argv[2];
}

void validate_opts() {
  if (!boost::filesystem::exists(dir_in)) {
    shutdown_with_error("error: %s does not exist.", &dir_in);
  } else if (!boost::filesystem::is_directory(dir_in)) {
    shutdown_with_error("error: %s is not a directory.", &dir_in);
  }

  if (!boost::filesystem::exists(dir_out)) {
    if (!boost::filesystem::create_directory(dir_out)) {
      shutdown_with_error("error: creating %s was failure.", &dir_out);
    }
  } else if (!boost::filesystem::is_directory(dir_out)) {
    shutdown_with_error("error: %s is not a directory.", &dir_out);
  }
}

int main(int argc, char *argv[]) {
  parse_opts(argc, argv);
  validate_opts();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in1(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);

  auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>();

  std::vector<int> viewports;
  splitVisualizer(*viewer, 3, 2, viewports);

  auto itr = std::make_shared<std::vector<int>::iterator>(viewports.begin());

  // Registration with ICP (Iterative Closed Point)
  each_files(dir_in, [&](bfs::directory_entry entry) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in2(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
    auto path = entry.path();

    pcl::io::loadPCDFile(path.string(), *cloud_tmp);

    viewer->addPointCloud(cloud_tmp, path.string(), **itr);
    (*itr)++;

    if (cloud_in1->empty()) {
      downsample(cloud_tmp, *cloud_in1, 0.02f);
      pcl::copyPointCloud(*cloud_tmp, *cloud_out);
    } else {
      std::cout << path.string() << std::endl;
      downsample(cloud_tmp, *cloud_in2, 0.02f);

      pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
      icp.setInputTarget(cloud_in1);
      icp.setInputCloud(cloud_in2);
      icp.setMaximumIterations(100);
      icp.align(*cloud_out);

      std::cout << "has converged: " << icp.hasConverged()
        << " score: " << icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;
    }

    viewer->addPointCloud(cloud_out, path.string() + ".tansformed", viewports.back());
  });

  viewer->setSize(1280, 1024);

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }

  return 0;
}
