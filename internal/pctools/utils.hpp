//
// Created by Masayuki IZUMI on 5/11/16.
//
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

namespace bfs = boost::filesystem;
namespace pc = pcl::console;

namespace pctools {

void shutdown_with_error(const char* format, ...) {
  va_list ap;
  va_start(ap, format);
  pc::print_error(format, ap);
  va_end(ap);
  exit(-1);
}

void each_files(std::string path, std::function<void(bfs::directory_entry)> proc) {
  for (auto file : boost::make_iterator_range(bfs::directory_iterator(path), bfs::directory_iterator())) {
    proc(file);
  }
}

template <typename PointT>
void downsample(typename pcl::PointCloud<PointT>::Ptr cloud_in, pcl::PointCloud<PointT> &cloud_out, float voxelSize) {
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud_in);
  sor.setLeafSize(voxelSize, voxelSize, voxelSize);
  sor.filter(cloud_out);
}

void splitVisualizer(pcl::visualization::PCLVisualizer &visualizer, int width, int height, std::vector<int> &viewports) {
  double x_step = 1.0 / width, y_step = 1.0 / height;
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      int idx = height * i + j + 1;
      viewports.push_back(idx);
      visualizer.createViewPort(i * x_step, 1 - (j + 1) * y_step, (i + 1) * x_step, 1 - j * y_step, idx);
      std::stringstream ss;
      ss << idx;
      visualizer.addText(ss.str(), 15, 15, ss.str() + ".viewport", idx);
    }
  }
}
}

