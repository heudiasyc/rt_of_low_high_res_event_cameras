#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_msgs/msg/event_packet.hpp>
#include <rt_of_event_msgs/msg/event.hpp>
#include <rt_of_event_msgs/msg/event_array.hpp>

using EventPacket = event_camera_msgs::msg::EventPacket;
using EventArray = rt_of_event_msgs::msg::EventArray;
using Event = rt_of_event_msgs::msg::Event;

class DavisAdapter : public rclcpp::Node,
                     public event_camera_codecs::EventProcessor
{
public:
  DavisAdapter() : Node("davis_event_adapter")
  {
    publisher_ = create_publisher<EventArray>(
      "/prophesee/camera/cd_events_buffer", 100);

    subscriber_ = create_subscription<EventPacket>(
      "/event_camera/events",
      rclcpp::SensorDataQoS(),
      [this](EventPacket::ConstSharedPtr msg) {
        processPacket(msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "DAVIS adapter ready: EventPacket -> EventArray");
  }

  void eventCD(
    uint64_t timestamp,
    uint16_t x,
    uint16_t y,
    uint8_t polarity) override
  {
    if (x >= output_.width || y >= output_.height) {
      return;
    }

    Event e;
    e.x = x;
    e.y = y;
    e.polarity = (polarity != 0);

    e.ts.sec = static_cast<int32_t>(
      timestamp / 1000000000ULL);

    e.ts.nanosec = static_cast<uint32_t>(
      timestamp % 1000000000ULL);

    output_.events.push_back(e);
  }

  bool eventExtTrigger(
    uint64_t,
    uint8_t,
    uint8_t) override
  {
    return true;
  }

  void finished() override {}

  void rawData(const char *, size_t) override {}

private:
  void processPacket(EventPacket::ConstSharedPtr msg)
  {
    auto decoder = decoder_factory_.getInstance(*msg);

    if (!decoder) {
      RCLCPP_ERROR(
        get_logger(),
        "Unsupported encoding: %s",
        msg->encoding.c_str());
      return;
    }

    output_ = EventArray();
    output_.header = msg->header;
    output_.width = msg->width;
    output_.height = msg->height;

    decoder->decode(*msg, this);

    if (!output_.events.empty()) {
      publisher_->publish(output_);
    }

    ++packets_;

    if (packets_ % 100 == 0) {
      RCLCPP_INFO(
        get_logger(),
        "Decoded packet %zu, events=%zu, resolution=%ux%u",
        packets_,
        output_.events.size(),
        output_.width,
        output_.height);
    }
  }

  event_camera_codecs::DecoderFactory<
    EventPacket, DavisAdapter> decoder_factory_;

  EventArray output_;

  rclcpp::Publisher<EventArray>::SharedPtr publisher_;
  rclcpp::Subscription<EventPacket>::SharedPtr subscriber_;

  size_t packets_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DavisAdapter>());
  rclcpp::shutdown();
  return 0;
}
