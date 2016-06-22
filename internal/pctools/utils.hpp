//
// Created by Masayuki IZUMI on 5/11/16.
//

#include <opencv/cv.hpp>

#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

namespace bfs = boost::filesystem;
namespace pc = pcl::console;

namespace pctools {

void transformByCalibrationConfig(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud_in,
                                  const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out,
                                  const std::string yamlfile,
                                  const std::string serial,
                                  const std::string prefix = "kinect2_" ) {
  if (serial == "") {
    throw std::runtime_error( "loadConfig: invalid serial number" );
  }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);

  cv::Mat cameraMatrix, rot, tr;
  Eigen::Matrix4f calibMatrix = Eigen::Matrix4f::Identity();
  cv::FileStorage fs(yamlfile, cv::FileStorage::READ);


  if (fs[prefix + serial].type() == cv::FileNode::NONE) {
    throw std::runtime_error( "loadConfig: no entry" );
  }

  fs[prefix + serial]["camera_matrix"]            >> cameraMatrix;
//  fs[prefix + serial]["distortion_coefficients"]  >> distCoeffs;
  fs[prefix + serial]["rotation"]                 >> rot;
  fs[prefix + serial]["translation"]              >> tr;
  fs.release();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      calibMatrix(i, j) = rot.at<double>(i, j);
    }
  }
  for (int i = 0; i < 3; i++) {
    calibMatrix(i, 3) = tr.at<double>(i);
  }

  auto fx = cameraMatrix.ptr<double>(0)[0];
  auto fy = cameraMatrix.ptr<double>(0)[4];
  auto cx = cameraMatrix.ptr<double>(0)[2];
  auto cy = cameraMatrix.ptr<double>(0)[5];

  std::cout << "fx=" << fx << ","
    << "fy=" << fy << ","
    << "cx=" << cx << ","
    << "cy=" << cy << std::endl;
  std::cout << "calib: " <<  calibMatrix << std::endl;
  std::cout << "calib inverse: " << calibMatrix.inverse() << std::endl;

  for (auto point : cloud_in->points) {
    pcl::PointXYZRGBA newPoint;
    newPoint.x = (point.x - cx) / fx;
    newPoint.y = (point.y - cy) / fy;
    newPoint.z = point.z;
    newPoint.r = point.r;
    newPoint.g = point.g;
    newPoint.b = point.b;
    newPoint.a = point.a;
    cloud_tmp->points.push_back(newPoint);
//    cloud_tmp->points.push_back(point);
  }

  calibMatrix = calibMatrix.inverse();

  pcl::transformPointCloud(*cloud_tmp, *cloud_out, calibMatrix);
}

Eigen::Matrix4f loadConfig(const std::string yamlfile, const std::string serial, std::string prefix="kinect2_") {
  if (serial == "") {
    throw std::runtime_error( "loadConfig: invalid serial number" );
  }

  cv::Mat rot, tr;
//  cv::Mat cameraMatrix, distCoeffs, Rmat, T;
  Eigen::Matrix4f tmpMatrix    = Eigen::Matrix4f::Identity();

//  cameraMatrix  = cv::Mat::eye(3, 3, CV_64FC1);
//  distCoeffs    = cv::Mat(5, 1, CV_64FC1, cv::Scalar::all(0));
//  rot           = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
//  tr            = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0));

  std::cout << "serial: " << serial << std::endl;
  cv::FileStorage fs(yamlfile, cv::FileStorage::READ);

  if (fs[prefix + serial].type() == cv::FileNode::NONE) {
    throw std::runtime_error( "loadConfig: no entry" );
  }

//  fs[prefix + serial]["camera_matrix"]            >> cameraMatrix;
//  fs[prefix + serial]["distortion_coefficients"]  >> distCoeffs;
  fs[prefix + serial]["rotation"]                 >> rot;
  fs[prefix + serial]["translation"]              >> tr;
  fs.release();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      tmpMatrix(i, j) = rot.at<double>(i, j);
    }
  }
  for (int i = 0; i < 3; i++) {
    tmpMatrix(i, 3) = tr.at<double>(i);
  }

  std::cout << "calib: " << tmpMatrix << std::endl;
  std::cout << "calib inverse: " << tmpMatrix.inverse() << std::endl;

  return tmpMatrix.inverse();
}

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
void downsample(typename pcl::PointCloud<PointT>::ConstPtr cloud_in, pcl::PointCloud<PointT> &cloud_out, float voxelSize) {
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

template <typename PointT>
void removeOutlier(const typename pcl::PointCloud<PointT>::ConstPtr cloud_in, typename pcl::PointCloud<PointT> &cloud_out, int mean_k, double stddev_mult_thr) {
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud_in);
  sor.setMeanK(mean_k);
  sor.setStddevMulThresh(stddev_mult_thr);
  sor.filter(cloud_out);
}

template <typename PointT>
void extractDifference(const typename pcl::PointCloud<PointT>::ConstPtr cloud_in1,
                       const typename pcl::PointCloud<PointT>::ConstPtr cloud_in2,
                       std::vector<int> &newPointIdxVector,
                       double resolution,
                       int noise_filter) {
  pcl::octree::OctreePointCloudChangeDetector<PointT> octree(resolution);
  octree.setInputCloud(cloud_in1);
  octree.addPointsFromInputCloud();
  octree.switchBuffers();
  octree.setInputCloud(cloud_in2);
  octree.addPointsFromInputCloud();
  octree.getPointIndicesFromNewVoxels(newPointIdxVector, noise_filter);
}

template <typename PointT>
void extractDifference(const typename pcl::PointCloud<PointT>::ConstPtr cloud_in1,
                       const typename pcl::PointCloud<PointT>::ConstPtr cloud_in2,
                       typename pcl::PointCloud<PointT>::Ptr &cloud_out,
                       double resolution,
                       int noise_filter) {
  std::vector<int> newPointIdxVector;
  extractDifference<PointT>(cloud_in1, cloud_in2, newPointIdxVector, resolution, noise_filter);

  for (auto i : newPointIdxVector) {
    cloud_out->points.push_back(cloud_in2->points[i]);
  }
}
}

