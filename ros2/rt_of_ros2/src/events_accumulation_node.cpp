#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rt_of_event_msgs/msg/event_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>

#include <deque>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using Event = rt_of_event_msgs::msg::Event;
using EventArray = rt_of_event_msgs::msg::EventArray;

static int64_t event_time_ns(const Event& e)
{
  return static_cast<int64_t>(e.ts.sec) * 1000000000LL +
         static_cast<int64_t>(e.ts.nanosec);
}

class EventsAccumulationNode : public rclcpp::Node
{
public:
  EventsAccumulationNode()
    : Node("events_accumulation_node")
  {
    const int window_ms =
      declare_parameter<int>("accumulation_window_ms", 15);

    if (window_ms <= 0)
      throw std::runtime_error(
        "accumulation_window_ms must be positive");

    window_ns_ = static_cast<int64_t>(window_ms) * 1000000LL;

    const std::string input_topic =
      declare_parameter<std::string>(
        "input_topic",
        "/prophesee/camera/cd_events_buffer");

    const std::string output_topic =
      declare_parameter<std::string>(
        "output_topic", "/edge_image");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

    publisher_ =
      create_publisher<sensor_msgs::msg::Image>(
        output_topic, qos);

    subscriber_ =
      create_subscription<EventArray>(
        input_topic, qos,
        [this](EventArray::ConstSharedPtr msg) {
          process(msg);
        });

    RCLCPP_INFO(
      get_logger(),
      "Event accumulation ready: input=%s output=%s window=%d ms",
      input_topic.c_str(),
      output_topic.c_str(),
      window_ms);
  }

private:
  void process(EventArray::ConstSharedPtr msg)
  {
    if (msg->width == 0 || msg->height == 0)
      return;

    if (width_ != static_cast<int>(msg->width) ||
        height_ != static_cast<int>(msg->height)) {
      if (!buffer_.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "Camera dimensions changed; clearing event buffer");
        buffer_.clear();
      }

      width_ = static_cast<int>(msg->width);
      height_ = static_cast<int>(msg->height);
    }

    for (const auto& event : msg->events) {
      if (!buffer_.empty() &&
          event_time_ns(event) < event_time_ns(buffer_.back())) {
        RCLCPP_WARN(
          get_logger(),
          "Out-of-order event timestamp; clearing buffer");
        buffer_.clear();
      }

      buffer_.push_back(event);
    }

    if (buffer_.empty())
      return;

    // Match the original replay node's windowing logic:
    // accumulate until the last event exceeds the first
    // event by at least one accumulation window.
    if (event_time_ns(buffer_.back()) <
        event_time_ns(buffer_.front())) {
      buffer_.clear();
      return;
    }

    while (!buffer_.empty() &&
           event_time_ns(buffer_.back()) -
             event_time_ns(buffer_.front()) >= window_ns_) {

      std::vector<Event> window;
      window.reserve(300000);

      const int64_t first_ns =
        event_time_ns(buffer_.front());

      while (!buffer_.empty()) {
        Event e = buffer_.front();
        buffer_.pop_front();
        window.push_back(e);

        if (event_time_ns(e) - first_ns >= window_ns_)
          break;
      }

      cv::Mat image =
        cv::Mat::zeros(height_, width_, CV_8UC1);

      for (const auto& e : window) {
        if (e.x < width_ && e.y < height_)
          image.at<uint8_t>(e.y, e.x) = 255;
      }

      std_msgs::msg::Header header = msg->header;

      if (!window.empty())
        header.stamp = window.back().ts;

      publisher_->publish(
        *cv_bridge::CvImage(
          header, "mono8", image).toImageMsg());

      ++frames_;

      if (frames_ <= 3 || frames_ % 50 == 0) {
        RCLCPP_INFO(
          get_logger(),
          "Frame %zu: events=%zu, edge_nonzero=%d",
          frames_, window.size(),
          cv::countNonZero(image));
      }
    }
  }

  int width_ = 0;
  int height_ = 0;
  int64_t window_ns_ = 15000000LL;
  size_t frames_ = 0;

  std::deque<Event> buffer_;

  rclcpp::Publisher<
    sensor_msgs::msg::Image>::SharedPtr publisher_;

  rclcpp::Subscription<EventArray>::SharedPtr subscriber_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<EventsAccumulationNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("events_accumulation_node"),
      "%s", e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
