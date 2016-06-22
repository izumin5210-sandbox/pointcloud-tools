//
// Created by Masayuki IZUMI on 5/8/16.
//

#include <OpenNI.h>

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>
#include <thread>

#include "pctools/ni_device.hpp"

namespace pc = pcl::console;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

std::string onifile;
std::string yamlfile;
std::string dir_out;

void shutdown_with_error(const char* format, ...) {
  va_list ap;
  va_start(ap, format);
  pc::print_error(format, ap);
  va_end(ap);
  openni::OpenNI::shutdown();
  exit(-1);
}

void print_usage(char *argv[]) {
  std::cout << "Usage: " << argv[0] << "[options] <path-to-oni-file>\n"
    << "Options:\n"
    << "\n"
    << "  -o <directory>\tPlace the output into <directory>\n"
    << "  -y <yamlfile>\tPath to calibration configuration file\n"
    << std::endl;
}

bool parse_opts(int argc, char *argv[]) {
  if (pc::find_switch(argc, argv, "-h") || pc::find_switch(argc, argv, "--help")) {
    print_usage(argv);
    return false;
  }

  if (pc::parse_argument(argc, argv, "-o", dir_out) == -1) {
    shutdown_with_error("error: -o option is required.");
  }

  if (pc::parse_argument(argc, argv, "-y", yamlfile) == -1) {
    shutdown_with_error("error: -y option is required.");
  }

  auto files = pc::parse_file_extension_argument(argc, argv, "oni");
  if (files.size() < 1) {
    shutdown_with_error("error: oni file is required.");
  } else if (files.size() > 1) {
    shutdown_with_error("error: oni files are too much.");
  }
  onifile = argv[files.front()];

  return true;
}

void validate_opts() {
  if (!boost::filesystem::exists(onifile)) {
    shutdown_with_error("error: %s does not exist.", &onifile);
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
  if (!parse_opts(argc, argv)) {
    return 0;
  }

  validate_opts();

  pctools::NiDevice device(onifile, yamlfile);

  auto listener = [&](const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud){
    PointCloudPtr cloud_filtered_nan(new PointCloud);
    std::vector<int> nan_indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_filtered_nan, nan_indices);

    std::ostringstream filestream;
    filestream << dir_out << "/" << cloud->header.stamp << ".pcd";
    std::cout << filestream.str() << " (" << cloud_filtered_nan->size() << " points)" << std::endl;
    pcl::io::savePCDFile(filestream.str(), *cloud_filtered_nan, true);
  };

  int count = 0;

  while (++count) {
    device.update(listener);
  }

  return 0;
}
