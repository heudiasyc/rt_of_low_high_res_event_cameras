#include <rclcpp/rclcpp.hpp>
#include <rt_of_event_msgs/msg/event_array.hpp>
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

using EventArray = rt_of_event_msgs::msg::EventArray;
using Clock = std::chrono::steady_clock;

class EventsH5Node : public rclcpp::Node
{
public:
  EventsH5Node() : Node("events_h5_node")
  {
    auto path = declare_parameter<std::string>("input_file", "");
    auto topic = declare_parameter<std::string>(
      "output_topic", "/dsec/events");

    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    batch_size_ = declare_parameter<int>("batch_size", 10000);
    replay_rate_ = declare_parameter<double>("replay_rate", 1.0);
    max_duration_ = declare_parameter<double>("max_duration_sec", 0.0);
    int delay = declare_parameter<int>("startup_delay_ms", 3000);

    if (path.empty() || width_ <= 0 || height_ <= 0 ||
        batch_size_ <= 0 || !std::isfinite(replay_rate_) ||
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

    pub_ = create_publisher<EventArray>(
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
    const hsize_t count =
      std::min<hsize_t>(1000000, total_ - position_);

    read_chunk(ds_x_, xs_,
      H5::PredType::NATIVE_UINT16, position_, count);

    read_chunk(ds_y_, ys_,
      H5::PredType::NATIVE_UINT16, position_, count);

    read_chunk(ds_t_, ts_,
      H5::PredType::NATIVE_UINT64, position_, count);

    read_chunk(ds_p_, ps_,
      H5::PredType::NATIVE_UINT8, position_, count);

    chunk_index_ = 0;
  }

  void tick()
  {
    if (Clock::now() < ready_at_)
      return;

    if (!started_) {
      replay_start_ = Clock::now();
      started_ = true;
    }

    double elapsed =
      std::chrono::duration<double>(
        Clock::now() - replay_start_).count();

    const double available_us =
      elapsed * replay_rate_ * 1000000.0;

    EventArray msg;
    msg.width = width_;
    msg.height = height_;
    msg.header.frame_id = "dsec_left";
    msg.events.reserve(batch_size_);

    bool finished = false;

    while (position_ < total_ &&
           msg.events.size() < static_cast<size_t>(batch_size_))
    {
      if (chunk_index_ >= ts_.size())
        load_chunk();

      const auto i = chunk_index_;

      if (ts_[i] < first_t_ ||
          (position_ > 0 && ts_[i] < previous_t_))
        throw std::runtime_error("Unordered timestamps");

      const uint64_t relative_us = ts_[i] - first_t_;

      if (max_duration_ > 0 &&
          relative_us >= max_duration_ * 1000000.0) {
        finished = true;
        break;
      }

      if (relative_us > available_us)
        break;

      if (ts_[i] > static_cast<uint64_t>(
          std::numeric_limits<int64_t>::max()))
        throw std::runtime_error("Timestamp overflow");

      const int64_t raw_us = static_cast<int64_t>(ts_[i]);

      if (t_offset_ > 0 &&
          raw_us > std::numeric_limits<int64_t>::max() - t_offset_)
        throw std::runtime_error("Timestamp offset overflow");

      const int64_t absolute_us = raw_us + t_offset_;

      if (absolute_us < 0 ||
          absolute_us / 1000000LL >
            std::numeric_limits<int32_t>::max())
        throw std::runtime_error("Timestamp outside ROS2 range");

      if (xs_[i] < width_ && ys_[i] < height_) {
        rt_of_event_msgs::msg::Event event;

        event.x = xs_[i];
        event.y = ys_[i];
        event.polarity = ps_[i] != 0;

        event.ts.sec =
          static_cast<int32_t>(absolute_us / 1000000LL);

        event.ts.nanosec =
          static_cast<uint32_t>(
            (absolute_us % 1000000LL) * 1000LL);

        msg.events.push_back(event);
      }

      previous_t_ = ts_[i];
      ++chunk_index_;
      ++position_;
    }

    if (!msg.events.empty()) {
      msg.header.stamp = msg.events.back().ts;
      pub_->publish(msg);

      published_events_ += msg.events.size();
      ++published_batches_;

      if (published_batches_ == 1 ||
          published_batches_ % 500 == 0) {
        RCLCPP_INFO(
          get_logger(),
          "Published events=%llu, batches=%llu",
          static_cast<unsigned long long>(published_events_),
          static_cast<unsigned long long>(published_batches_));
      }
    }

    if (finished || position_ >= total_) {
      timer_->cancel();

      RCLCPP_INFO(
        get_logger(),
        "Finished HDF5 replay: events=%llu, batches=%llu",
        static_cast<unsigned long long>(published_events_),
        static_cast<unsigned long long>(published_batches_));
    }
  }

  H5::H5File file_;
  H5::DataSet ds_x_, ds_y_, ds_t_, ds_p_;

  std::vector<uint16_t> xs_, ys_;
  std::vector<uint64_t> ts_;
  std::vector<uint8_t> ps_;

  hsize_t total_ = 0;
  hsize_t position_ = 0;
  size_t chunk_index_ = 0;

  uint64_t first_t_ = 0;
  uint64_t previous_t_ = 0;
  int64_t t_offset_ = 0;

  uint64_t published_events_ = 0;
  uint64_t published_batches_ = 0;

  int width_, height_, batch_size_;
  double replay_rate_, max_duration_;

  bool started_ = false;
  Clock::time_point ready_at_, replay_start_;

  rclcpp::Publisher<EventArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  H5::Exception::dontPrint();

  try {
    rclcpp::spin(std::make_shared<EventsH5Node>());
  }
  catch (const H5::Exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("events_h5_node"),
      "HDF5 error: %s", e.getDetailMsg().c_str());

    rclcpp::shutdown();
    return 1;
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("events_h5_node"),
      "%s", e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
