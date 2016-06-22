//
// Created by Masayuki IZUMI on 5/9/16.
//

#include <pcl/point_cloud.h>
#include <OpenNI.h>

template <typename PointT>
class VideoFrameConverter<PointT> {
public:
  VideoFrameConverter(
      const openni::VideoStream *color_video_stream,
      const openni::VideoStream *depth_video_stream,
      const unsigned width,
      const unsigned height
  )
      : width_(width)
      , height_(height)
  {
    int frameWidth = depth_video_stream->getVideoMode().getResolutionX();
    float hFov = depth_video_stream->getHorizontalFieldOfView();
    float fx = frameWidth / (2.0f * tan (hFov / 2.0f));
    float cx = (depth_video_stream->getSensorInfo().getSupportedVideoModes())


  }


private:
  const unsigned width_;
  const unsigned height_;

  const float fx_inv;
  const float fy_inv;
};

//convertToXYZRGBPointCloud(
//    const openni::VideoFrame *rgb_frame_ref,
//    const openni::VideoFrameRef *depth_frame_ref,
//    const unsigned width,
//    const unsigned height
//) {
//  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//
//  cloud->header.seq = depth_frame_ref->getFrameIndex();
//  cloud->header.stamp = depth_frame_ref->getTimestamp();
//  // https://github.com/PointCloudLibrary/pcl/blob/pcl-1.8.0rc2/io/src/openni2_grabber.cpp#L103
//  cloud->header.frame_id = "/openni2_rgb_optical_frame";
//  cloud->height = height;
//  cloud->width = width;
//}

