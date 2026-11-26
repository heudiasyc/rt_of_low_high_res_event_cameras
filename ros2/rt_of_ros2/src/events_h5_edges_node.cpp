#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "rt_of_ros2/event_accumulator.hpp"
#include "rt_of_ros2/rectification_h5.hpp"
#include <H5Cpp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using Image = sensor_msgs::msg::Image;
using Clock = std::chrono::steady_clock;

class EventsH5EdgesNode : public rclcpp::Node
{
public:
  EventsH5EdgesNode() : Node("events_h5_edges_node")
  {
    auto path = declare_parameter<std::string>("input_file", "");
    auto calibration_file =
      declare_parameter<std::string>("calibration_file", "");
    auto topic = declare_parameter<std::string>(
      "output_topic", "/edge_image");

    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    window_ms_ = declare_parameter<int>(
      "accumulation_window_ms", 25);
    replay_rate_ = declare_parameter<double>("replay_rate", 1.0);
    max_duration_ = declare_parameter<double>("max_duration_sec", 0.0);
    int delay = declare_parameter<int>("startup_delay_ms", 3000);

    if (path.empty() || width_ <= 0 || height_ <= 0 ||
        window_ms_ <= 0 || !std::isfinite(replay_rate_) ||
        replay_rate_ <= 0 || !std::isfinite(max_duration_) ||
        max_duration_ < 0 || delay < 0)
      throw std::runtime_error("Invalid replay parameters");

    // Reuse the original HDF5 dataset structure.
    file_ = H5::H5File(path, H5F_ACC_RDONLY);

    ds_x_ = file_.openDataSet("/events/x");
    ds_y_ = file_.openDataSet("/events/y");
    ds_t_ = file_.openDataSet("/events/t");
    ds_p_ = file_.openDataSet("/events/p");

    auto offset_ds = file_.openDataSet("/t_offset");
    offset_ds.read(&t_offset_, H5::PredType::NATIVE_INT64);

    total_ = dataset_size(ds_x_);

    if (total_ == 0 ||
        dataset_size(ds_y_) != total_ ||
        dataset_size(ds_t_) != total_ ||
        dataset_size(ds_p_) != total_)
      throw std::runtime_error("Invalid HDF5 dataset dimensions");

    accumulator_ = std::make_unique<rt_of_ros2::EventAccumulator>(
      width_, height_, static_cast<uint64_t>(window_ms_) * 1000);

    if (!calibration_file.empty()) {
      rectification_map_ = rt_of_ros2::load_rectification_h5(
        calibration_file, width_, height_);

      RCLCPP_INFO(
        get_logger(), "Rectification enabled: %s",
        calibration_file.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Rectification disabled");
    }

    pub_ = create_publisher<Image>(
      topic, rclcpp::QoS(rclcpp::KeepLast(100)).reliable());

    load_chunk();
    first_t_ = ts_[0];

    ready_at_ = Clock::now() +
      std::chrono::milliseconds(delay);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      [this]() { tick(); });

    RCLCPP_INFO(
      get_logger(),
      "HDF5 ready: %dx%d, events=%llu, offset=%lld, rate=%.2f",
      width_, height_,
      static_cast<unsigned long long>(total_),
      static_cast<long long>(t_offset_),
      replay_rate_);
  }

private:
  static hsize_t dataset_size(H5::DataSet& ds)
  {
    auto space = ds.getSpace();

    if (space.getSimpleExtentNdims() != 1)
      throw std::runtime_error("Expected one-dimensional dataset");

    hsize_t n;
    space.getSimpleExtentDims(&n);
    return n;
  }

  // Read only a subset of the HDF5 dataset.
  template<typename T>
  void read_chunk(
    H5::DataSet& ds,
    std::vector<T>& buffer,
    const H5::PredType& type,
    hsize_t start,
    hsize_t count)
  {
    buffer.resize(count);

    auto file_space = ds.getSpace();

    file_space.selectHyperslab(
      H5S_SELECT_SET, &count, &start);

    H5::DataSpace memory_space(1, &count);

    ds.read(
      buffer.data(), type,
      memory_space, file_space);
  }

  void load_chunk()
  {
    const auto read_start = Clock::now();
    const hsize_t count =
      std::min<hsize_t>(8000000, total_ - position_);

    read_chunk(ds_x_, xs_,
      H5::PredType::NATIVE_UINT16, position_, count);

    read_chunk(ds_y_, ys_,
      H5::PredType::NATIVE_UINT16, position_, count);

    read_chunk(ds_t_, ts_,
      H5::PredType::NATIVE_UINT32, position_, count);

    chunk_index_ = 0;

    const double read_ms =
      std::chrono::duration<double, std::milli>(
        Clock::now() - read_start).count();

    ++chunks_read_;
    total_read_ms_ += read_ms;

    if (chunks_read_ <= 3 || chunks_read_ % 20 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "HDF5 chunk: events=%llu, read=%.2f ms, mean=%.2f ms",
        static_cast<unsigned long long>(count),
        read_ms,
        total_read_ms_ / chunks_read_);
    }
  }


  void tick()
  {
    if (Clock::now() < ready_at_)
      return;

    if (!started_) {
      replay_start_ = Clock::now();
      started_ = true;
    }

    // Publish the completed image only when its timestamp is due.
    if (frame_ready_) {
      const double elapsed =
        std::chrono::duration<double>(
          Clock::now() - replay_start_).count();

      const double due =
        static_cast<double>(frame_t_ - first_t_) /
        (1000000.0 * replay_rate_);

      if (elapsed < due)
        return;

      const int64_t raw_us = static_cast<int64_t>(frame_t_);

      if (t_offset_ > 0 &&
          raw_us > std::numeric_limits<int64_t>::max() - t_offset_)
        throw std::runtime_error("Timestamp offset overflow");

      const int64_t absolute_us = raw_us + t_offset_;

      if (absolute_us < 0 ||
          absolute_us / 1000000LL >
            std::numeric_limits<int32_t>::max())
        throw std::runtime_error("Timestamp outside ROS2 range");

      std_msgs::msg::Header header;
      header.frame_id = "dsec_left";
      header.stamp.sec =
        static_cast<int32_t>(absolute_us / 1000000LL);
      header.stamp.nanosec =
        static_cast<uint32_t>(
          (absolute_us % 1000000LL) * 1000LL);

      pub_->publish(
        *cv_bridge::CvImage(
          header, "mono8", accumulator_->image()).toImageMsg());

      ++frames_;

      if (frames_ <= 3 || frames_ % 50 == 0) {
        RCLCPP_INFO(
          get_logger(),
          "Frame %llu: events=%llu, edge_nonzero=%d",
          static_cast<unsigned long long>(frames_),
          static_cast<unsigned long long>(
            accumulator_->event_count()),
          cv::countNonZero(accumulator_->image()));
      }

      accumulator_->reset();
      frame_ready_ = false;
    }

    // Read events directly into the binary edge image.
    while (position_ < total_ && rclcpp::ok()) {

      if (chunk_index_ >= ts_.size())
        load_chunk();

      const auto i = chunk_index_;
      const uint64_t t = ts_[i];

      if (t < first_t_ ||
          (position_ > 0 && t < previous_t_))
        throw std::runtime_error("Unordered timestamps");

      if (t > static_cast<uint64_t>(
          std::numeric_limits<int64_t>::max()))
        throw std::runtime_error("Timestamp overflow");

      if (max_duration_ > 0 &&
          t - first_t_ >= max_duration_ * 1000000.0)
        break;

      bool completed = false;

      if (xs_[i] < width_ && ys_[i] < height_) {
        int x = xs_[i];
        int y = ys_[i];

        if (!rectification_map_.empty()) {
          const auto point =
            rectification_map_.at<cv::Point2i>(y, x);

          x = point.x;
          y = point.y;

          if (x < 0 || x >= width_ ||
              y < 0 || y >= height_) {
            ++rectification_rejected_;
          }
        }

        completed = accumulator_->add(x, y, t);
        ++processed_events_;
      } else {
        ++invalid_events_;
      }

      previous_t_ = t;
      ++chunk_index_;
      ++position_;

      if (completed) {
        frame_t_ = t;
        frame_ready_ = true;
        return;
      }
    }

    timer_->cancel();

    RCLCPP_INFO(
      get_logger(),
      "Rectification rejected events: %llu",
      static_cast<unsigned long long>(
        rectification_rejected_));

    RCLCPP_INFO(
      get_logger(),
      "Finished HDF5 replay: events=%llu, frames=%llu, "
      "invalid=%llu, remaining_events=%llu",
      static_cast<unsigned long long>(processed_events_),
      static_cast<unsigned long long>(frames_),
      static_cast<unsigned long long>(invalid_events_),
      static_cast<unsigned long long>(
        accumulator_->event_count()));
  }

  H5::H5File file_;
  H5::DataSet ds_x_, ds_y_, ds_t_, ds_p_;

  std::vector<uint16_t> xs_, ys_;
  std::vector<uint32_t> ts_;

  hsize_t total_ = 0;
  hsize_t position_ = 0;
  size_t chunk_index_ = 0;

  uint64_t first_t_ = 0;
  uint64_t previous_t_ = 0;
  int64_t t_offset_ = 0;

  uint64_t chunks_read_ = 0;
  double total_read_ms_ = 0.0;

  uint64_t processed_events_ = 0;
  uint64_t invalid_events_ = 0;
  uint64_t rectification_rejected_ = 0;
  cv::Mat rectification_map_;
  uint64_t frames_ = 0;
  uint64_t frame_t_ = 0;
  bool frame_ready_ = false;

  std::unique_ptr<rt_of_ros2::EventAccumulator> accumulator_;

  int width_, height_, window_ms_;
  double replay_rate_, max_duration_;

  bool started_ = false;
  Clock::time_point ready_at_, replay_start_;

  rclcpp::Publisher<Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  H5::Exception::dontPrint();

  try {
    rclcpp::spin(std::make_shared<EventsH5EdgesNode>());
  }
  catch (const H5::Exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("events_h5_edges_node"),
      "HDF5 error: %s", e.getDetailMsg().c_str());

    rclcpp::shutdown();
    return 1;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("events_h5_edges_node"),
      "%s", e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
