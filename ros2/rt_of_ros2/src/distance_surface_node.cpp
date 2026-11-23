#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

class DistanceSurfaceNode : public rclcpp::Node {
public:
  DistanceSurfaceNode() : Node("distance_surface_node") {
    formulation_ = declare_parameter<std::string>(
        "distance_surface_formulation", "exponential");

    if (formulation_ != "linear" &&
        formulation_ != "linear-bound" &&
        formulation_ != "logarithmic" &&
        formulation_ != "exponential") {
      throw std::runtime_error("Invalid distance surface formulation");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();

    publisher_ = create_publisher<sensor_msgs::msg::Image>(
        "distance_surface", qos);

    subscriber_ = create_subscription<sensor_msgs::msg::Image>(
        "denoised_filled_edge_image", qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          process(msg);
        });

    RCLCPP_INFO(
        get_logger(),
        "Distance surface node ready; formulation=%s",
        formulation_.c_str());
  }

private:
  void process(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      auto cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
      const cv::Mat& edges = cv_ptr->image;

      if (edges.empty()) {
        RCLCPP_WARN(get_logger(), "Empty input image");
        return;
      }

      // No edge pixels: distance to an edge is undefined.
      if (cv::countNonZero(edges) == 0) {
        RCLCPP_WARN(get_logger(), "No edges; skipping frame");
        return;
      }

      // OpenCV distanceTransform computes distance to zero pixels.
      // Our edges are nonzero, so invert the binary edge mask.
      cv::Mat binary;
      cv::threshold(edges, binary, 0, 255, cv::THRESH_BINARY);

      cv::Mat inverted;
      cv::bitwise_not(binary, inverted);

      cv::Mat distance;
      cv::distanceTransform(
          inverted,
          distance,
          cv::DIST_L2,
          cv::DIST_MASK_PRECISE,
          CV_32F);

      // Preserve the four formulations from the original project.
      constexpr float saturation = 6.0f;
      constexpr float alpha = saturation / 5.541f;

      if (formulation_ == "linear-bound") {
        cv::threshold(
            distance, distance, saturation,
            saturation, cv::THRESH_TRUNC);
      } else if (formulation_ == "logarithmic") {
        cv::log(distance + 1.0f, distance);
      } else if (formulation_ == "exponential") {
        cv::Mat scaled = -distance / alpha;
        cv::exp(scaled, distance);
        distance = 1.0f - distance;
      }

      cv::Mat normalized;
      cv::normalize(
          distance,
          normalized,
          0, 255,
          cv::NORM_MINMAX,
          CV_8U);

      auto output = cv_bridge::CvImage(
          msg->header, "mono8", normalized).toImageMsg();

      publisher_->publish(*output);

      ++frame_count_;
      RCLCPP_INFO(
          get_logger(),
          "Published distance frame %zu: %dx%d, nonzero=%d",
          frame_count_,
          normalized.cols,
          normalized.rows,
          cv::countNonZero(normalized));

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Processing failed: %s", e.what());
    }
  }

  std::string formulation_;
  size_t frame_count_ = 0;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<DistanceSurfaceNode>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
        rclcpp::get_logger("distance_surface_node"),
        "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
