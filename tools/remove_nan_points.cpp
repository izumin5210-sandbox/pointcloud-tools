//
// Created by Masayuki IZUMI on 5/11/16.
//

#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

namespace pc = pcl::console;
namespace bfs = boost::filesystem;

std::string dir_in;
std::string dir_out;

void shutdown_with_error(const char* format, ...) {
  va_list ap;
  va_start(ap, format);
  pc::print_error(format, ap);
  va_end(ap);
  exit(-1);
}

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

void each_files(std::string path, std::function<void(bfs::directory_entry)> proc) {
  for (auto file : boost::make_iterator_range(bfs::directory_iterator(path), bfs::directory_iterator())) {
    proc(file);
  }
}

int main(int argc, char *argv[]) {
  parse_opts(argc, argv);
  validate_opts();

  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGBA>);
  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);

  each_files(dir_in, [=](bfs::directory_entry entry) {
    auto path = entry.path();
    std::vector<int> indices;
    pcl::io::loadPCDFile(path.string(), *cloud_in);
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, indices);

    pcl::io::savePCDFile(dir_out + "/" + path.filename().string(), *cloud_out, true);

    std::cout << path.filename().string() << ":\t"
        << cloud_in->size() << "\t-> " << cloud_out->size()
        << "\t (" << indices.size() << " points were removed)" << std::endl;
  });
}