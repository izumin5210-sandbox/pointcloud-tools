//
// Created by Masayuki IZUMI on 5/9/16.
//

#include <OpenNI.h>


const float getFocalLength(openni::VideoStream *stream) const {
  int frameWidth = stream->getVideoMode().getResolutionX();
  float hFov = stream->getHorizontalFieldOfView();
  return frameWidth / (2.0f * tan (hFov / 2.0f));
}

const openni::VideoMode getDefaultVideoMode(openni::VideoStream *stream) const {
  auto modes = getSupportedVideoModes(stream);
  for (auto mode : modes) {
    if (mode.getResolutionX() == 640 && mode.getResolutionY() == 480 && mode.getFps() == 30) {
      return mode;
    }
  }
  return modes.get(0);
}

const std::vector<openni::VideoMode>& getSupportedVideoModes(openni::VideoStream *stream) const {
  std::vector<openni::VideoMode> modes;
  for (auto mode : stream->getSensorInfo().getSupportedVideoModes()) {
    modes.push_back(mode);
  }
  return modes;
}

