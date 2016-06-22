//
// Created by Masayuki IZUMI on 5/8/16.
//

#include <OpenNI.h>

typedef std::function<void(openni::VideoStream& steam)> StreamCallbackFunction;

class OpenNI2FrameListener : public openni::VideoStream::NewFrameListener {

  public:
    OpenNI2FrameListener(StreamCallbackFunction cb)
        : callback_(cb) {}

    OpenNI2FrameListener()
        : callback_(0) {}

    virtual ~OpenNI2FrameListener() override {}

    virtual void onNewFrame(openni::VideoStream& stream) override {
      if (callback_) {
        callback_(stream);
      }
    }

private:
    StreamCallbackFunction callback_;
};
