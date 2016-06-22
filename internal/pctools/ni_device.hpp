//
// Created by Masayuki IZUMI on 5/18/16.
//

#ifndef TOOLS_NI_DEVICE_H
#define TOOLS_NI_DEVICE_H

#include <regex>

#include <opencv2/opencv.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <OpenNI.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef std::function<void (const PointCloudConstPtr& cloud)> FrameListener;

namespace pctools {
class NiDevice {
public:

  NiDevice(const std::string uri, const std::string yamlfile)
      : uri_(uri),
        yamlfile_(yamlfile)
  {
    initialize();
  }

  ~NiDevice() {
    if (colorStream_.isValid()) {
      colorStream_.stop();
      colorStream_.destroy();
    }
    if (irStream_.isValid()) {
      depthStream_.stop();
      depthStream_.destroy();
    }
    if (irStream_.isValid()) {
      irStream_.stop();
      irStream_.destroy();
    }
    device_.close();
  }

  inline void initialize() {
    openni::Status status;

    status = openni::OpenNI::initialize();
    checkStatus(status, "openni::OpenNI::initialize() failed.");

    status = device_.open(uri_.c_str());
    checkStatus(status, "openni::Device::open() failed.");

    if (device_.isFile()) {
      initializeFilePlaybackControl();
      std::regex serialPattern_( R"(.+\.\d+_(\d+)\.oni$))" );
      std::smatch m;
      std::regex_match(uri_, m, serialPattern_);
      serial_ = m[1].str();

      if (serial_.length() > 0) {
        loadConfig();
      }
    } else {
      std::runtime_error("support only *.oni file.");
    }

    startStream(colorStream_, openni::SENSOR_COLOR);
    startStream(depthStream_, openni::SENSOR_DEPTH);
    startStream(irStream_, openni::SENSOR_IR);

    if (colorStream_.isValid() && depthStream_.isValid()) {
//      status = device_.setDepthColorSyncEnabled(true);
//      checkStatus(status, "openni::Device::setDepthColorSyncEnabled() failed.");
//      status = device_.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
//      checkStatus(status, "openni::Device::setImageRegistrationMode() failed.");
    }
  }

  void update(std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&)> listener) {
//  void update(FrameListener &listener) {
    PointCloudPtr cloud(new PointCloud);
    openni::Status status;
    openni::VideoFrameRef colorFrame, depthFrame, irFrame;
    cv::UMat colorImage;
    cv::UMat rawDepthImage;
    cv::UMat depthImage;
    cv::UMat irImage;


    if (colorStream_.isValid()) {
      status = colorStream_.readFrame(&colorFrame);
      checkStatus(status, "openni::VideoStream::readFrame() failed. (color stream)");
      colorImage = updateColorImage(colorFrame);

      cloud->header.stamp = colorFrame.getTimestamp();
      cloud->header.frame_id = colorFrame.getFrameIndex();
    }
    if (depthStream_.isValid()) {
      status = depthStream_.readFrame(&depthFrame);
      checkStatus(status, "openni::VideoStream::readFrame() failed. (depth stream)");
      depthImage = updateDepthImage(depthFrame);
    }
    if (irStream_.isValid()) {
      status = irStream_.readFrame(&irFrame);
      checkStatus(status, "openni::VideoStream::readFrame() failed. (ir stream)");
      irImage = updateIrImage(irFrame);
    }

    int width = colorImage.size().width;
    int height = colorImage.size().height;
    int dwidth = depthImage.size().width;

    std::cout << "width: " << width << "\n"
        << "height" << height << "\n"
        << "dwitdh" << dwidth << std::endl;

    cv::Mat cvColorImage = colorImage.getMat(cv::ACCESS_READ);
    cv::Mat cvDepthImage = depthImage.getMat(cv::ACCESS_READ);

    for (int y = 0; y < height; y++) {
      unsigned short* depth = (unsigned short*) cvDepthImage.ptr(y);
      cv::Vec3b* color = (cv::Vec3b*) cvColorImage.ptr(y);

      for (int x = 0; x < width; x++) {
        if (depth[x] != 0 && (color[x][0] != 0 || color[x][1] != 0 || color[x][2] != 0)) {
          PointT point;
          float xw, yw, zw;
          zw = depth[x];
          zw /= 1000;
          xw = (x - cx) * zw / fx;
          yw = (y - cy) * zw / fy;
          point.x = xw;
          point.y = yw;
          point.z = zw;
          point.b = color[x][0];
          point.g = color[x][1];
          point.r = color[x][2];

//          std::cout << point.x << ", " << point.y << ", " << point.z << std::endl;

          cloud->push_back(point);
        }
      }
    }

    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_calibrated(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud(*cloud, *cloud_calibrated, calibMatrix);

    std::cout << cloud_calibrated->header.stamp << std::endl;
    std::cout << cloud_calibrated->header.frame_id << std::endl;

    listener(cloud_calibrated);
  }

private:
  openni::Device device_;
  std::string uri_;
  std::string serial_;
  std::string yamlfile_;

  openni::VideoStream colorStream_;
  openni::VideoStream depthStream_;
  openni::VideoStream irStream_;

  Eigen::Matrix4f calibMatrix;
  float fx, fy, cx, cy;

  void checkStatus(const openni::Status status, const std::string msg) {
    if (status != openni::STATUS_OK) {
      throw std::runtime_error(msg);
    }
  }

  void initializeFilePlaybackControl() {
    openni::Status status;
    if (device_.getPlaybackControl()->isValid()) {
      status = device_.getPlaybackControl()->setSpeed(-1);
      checkStatus(status, "openni::PlaybackControl::setSpeed() failed.");

      status = device_.getPlaybackControl()->setRepeatEnabled(false);
      checkStatus(status, "openni::PlaybackControl::setRepeatEnabled() failed.");
    }
  }

  void loadConfig() {
    if (serial_ == "") {
      throw std::runtime_error( "loadConfig: invalid serial number" );
    }

    std::string prefix = "kinect2_";

    cv::Mat rot, tr, cameraMatrix;
    Eigen::Matrix4f tmpMatrix = Eigen::Matrix4f::Identity();

    cameraMatrix  = cv::Mat::eye(3, 3, CV_64FC1);
    rot           = cv::Mat(3, 3, CV_64FC1, cv::Scalar::all(0));
    tr            = cv::Mat(3, 1, CV_64FC1, cv::Scalar::all(0));

    std::cout << "serial: " << serial_ << std::endl;
    cv::FileStorage fs(yamlfile_, cv::FileStorage::READ);

    if (fs[prefix + serial_].type() == cv::FileNode::NONE) {
      throw std::runtime_error( "loadConfig: no entry" );
    }

    fs[prefix + serial_]["camera_matrix"]  >> cameraMatrix;
//  fs[prefix + serial_]["distortion_coefficients"]  >> distCoeffs;
    fs[prefix + serial_]["rotation"]       >> rot;
    fs[prefix + serial_]["translation"]    >> tr;
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

    calibMatrix = tmpMatrix.inverse();
    fx = cameraMatrix.ptr<double>(0)[0];
    fy = cameraMatrix.ptr<double>(0)[4];
    cx = cameraMatrix.ptr<double>(0)[2];
    cy = cameraMatrix.ptr<double>(0)[5];
  }


  void startStream(openni::VideoStream &stream, openni::SensorType type) {
    openni::Status status;
    if (device_.hasSensor(type)) {
      status = stream.create(device_, type);
//      checkStatus(status, "openni::VideoStream::create() failed. (" + type + ")");
//      const openni::Array<openni::VideoMode> *supportedVideoModes = &(stream.getSensorInfo().getSupportedVideoModes());
//      if (supportedVideoModes->getSize() == 0) {
//        throw std::runtime_errror("VideoMode failed. (" + type + ")");
//      }

//      for (auto mode : supportedVideoModes->Array()) {
//        printf("%dx%d at %dfps with %d format \r\n",
//               mode.getResolutionX(),
//               mode.getResolutionY(),
//               mode.getFps(),
//               mode.getPixelFormat());
//      }

      status = stream.start();
//      checkStatus(status, "openni::VideoStream::start() failed. (" + type + ")");
    }
  }

  cv::UMat updateColorImage(const openni::VideoFrameRef &colorFrame) {
    cv::UMat colorImage;
    cv::Mat(
        colorFrame.getHeight(),
        colorFrame.getWidth(),
        CV_8UC3,
        (unsigned char*) colorFrame.getData()
    ).copyTo(colorImage);
    cv::cvtColor(colorImage, colorImage, CV_RGB2BGR);
    return colorImage;
  }

  cv::UMat updateDepthImage(const openni::VideoFrameRef &depthFrame) {
    cv::UMat depthImage;
    cv::Mat(
        depthFrame.getHeight(),
        depthFrame.getWidth(),
        CV_16UC1,
        (unsigned char*) depthFrame.getData()
    ).copyTo(depthImage);
    cv::Rect roi(0, 0, 512, 424);
    return depthImage(roi);
  }

  cv::UMat updateIrImage(const openni::VideoFrameRef &irFrame) {
    cv::UMat irImage;
    cv::Mat(
        irFrame.getHeight(),
        irFrame.getWidth(),
        CV_16UC1,
        ((unsigned char*) irFrame.getData())
    ).copyTo(irImage);
    cv::cvtColor(irImage, irImage, CV_GRAY2RGB);
    return irImage;
  }
};
}


#endif //TOOLS_NI_DEVICE_H
