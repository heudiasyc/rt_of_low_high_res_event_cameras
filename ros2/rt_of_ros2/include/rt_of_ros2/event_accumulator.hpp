#pragma once

#include <opencv2/core.hpp>
#include <cstdint>
#include <stdexcept>

namespace rt_of_ros2
{

class EventAccumulator
{
public:
  EventAccumulator(int width, int height, uint64_t window_us)
    : width_(width),
      height_(height),
      window_us_(window_us)
  {
    if (width <= 0 || height <= 0 || window_us == 0)
      throw std::runtime_error("Invalid accumulator parameters");

    image_ = cv::Mat::zeros(height, width, CV_8UC1);
  }

  bool add(int x, int y, uint64_t t)
  {
    if (!started_) {
      first_t_ = t;
      started_ = true;
    }

    if (t < first_t_)
      throw std::runtime_error("Unordered event timestamp");

    if (x >= 0 && x < width_ && y >= 0 && y < height_)
      image_.ptr<uint8_t>(y)[x] = 255;

    ++events_;

    return t - first_t_ >= window_us_;
  }

  const cv::Mat& image() const
  {
    return image_;
  }

  uint64_t event_count() const
  {
    return events_;
  }

  void reset()
  {
    image_.setTo(0);
    started_ = false;
    events_ = 0;
  }

private:
  int width_, height_;
  uint64_t window_us_;
  uint64_t first_t_ = 0;
  uint64_t events_ = 0;
  bool started_ = false;

  cv::Mat image_;
};

}
