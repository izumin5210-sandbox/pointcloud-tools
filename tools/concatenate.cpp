//
// Created by Masayuki IZUMI on 5/12/16.
//

#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <regex>
#include <pcl/filters/passthrough.h>

#include "pctools/utils.hpp"

namespace pc = pcl::console;
namespace bfs = boost::filesystem;

using namespace pctools;

std::string yamlfile;
std::string dir_in;
std::string dir_out;

void print_usage(char *argv[]) {
  std::cout << "Usage: " << argv[0] << " source_dir target_dir\n"
    << "\n"
    << "  -y  Path to calibration yaml file\n"
    << std::endl;
}

void parse_opts(int argc, char *argv[]) {
  if (pc::find_switch(argc, argv, "-h") || pc::find_switch(argc, argv, "--help")) {
    print_usage(argv);
    exit(0);
  }

  if (pc::parse_argument(argc, argv, "-y", yamlfile) == -1) {
    shutdown_with_error("error: yaml file is required.");
  }

//  if (argc < 2) {
//    shutdown_with_error("error: source directory and target directory is required.");
//  } else if (argc > 4) {
//    shutdown_with_error("error: arguments are too much.");
//  }

  dir_in = argv[argc - 2];
  dir_out = argv[argc - 1];
}

void validate_opts() {
  if (!bfs::exists(dir_in)) {
    shutdown_with_error("error: %s does not exist.", &dir_in);
  } else if (!bfs::is_directory(dir_in)) {
    shutdown_with_error("error: %s is not a directory.", &dir_in);
  }

  if (!bfs::exists(dir_out)) {
    if (!bfs::create_directory(dir_out)) {
      shutdown_with_error("error: creating %s was failure.", &dir_out);
    }
  } else if (!bfs::is_directory(dir_out)) {
    shutdown_with_error("error: %s is not a directory.", &dir_out);
  }
}

void filter(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud_in, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_out) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp1(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp2(new pcl::PointCloud<pcl::PointXYZRGBA>);
  removeOutlier(cloud_in, *cloud_tmp1, 50, 1.0);

  pcl::PassThrough<pcl::PointXYZRGBA> passY;
  passY.setFilterFieldName("y");
  passY.setInputCloud(cloud_tmp1);
  passY.setFilterLimits(-0.1f, 2.0f);
  passY.filter(*cloud_tmp2);

  pcl::PassThrough<pcl::PointXYZRGBA> passZ;
  passZ.setFilterFieldName("z");
  passZ.setInputCloud(cloud_tmp2);
  passZ.setFilterLimits(-0.6f, 0.6f);
  passZ.filter(*cloud_tmp1);

  pcl::PassThrough<pcl::PointXYZRGBA> passX;
  passX.setFilterFieldName("x");
  passX.setInputCloud(cloud_tmp1);
  passX.setFilterLimits(-0.6f, 0.6f);
  passX.filter(*cloud_out);
}

int main(int argc, char *argv[]) {
  parse_opts(argc, argv);
  validate_opts();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZRGBA>);

  // Registration with ICP (Iterative Closed Point)
  each_files(dir_in, [&](bfs::directory_entry entry) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
    auto path = entry.path();
    pcl::io::loadPCDFile(path.string(), *cloud_tmp);
    std::cout << path.string()
      << " (raw: " << cloud_tmp->size() << " points, filtered: " << std::flush;
    *cloud_merged += *cloud_tmp;
  });

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);

  filter(cloud_merged, cloud_out);

  std::list<std::string> list;
  boost::split(list, dir_in, boost::is_any_of("/"));

  std::stringstream ss;
  ss << dir_out << "/" << list.back() << ".pcd";

  pcl::io::savePCDFile(ss.str(), *cloud_out);

  std::cout << "Save as" << ss.str() << std::endl;

  return 0;
}
