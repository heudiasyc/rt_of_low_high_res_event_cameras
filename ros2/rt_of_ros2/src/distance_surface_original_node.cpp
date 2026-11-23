#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

// Original CUDA distance transform, adapted for ROS2.
cv::Mat distance_surface_gpu(const cv::Mat&, std::string);
void free_gpu_memory();

namespace {
constexpr int64_t INF = (1LL << 55);

// Step 1: exact horizontal distance to an edge in each row.
cv::Mat horizontal_distances(const cv::Mat& edges) {
  cv::Mat dx(edges.size(), CV_32S);
  for (int y = 0; y < edges.rows; ++y) {
    int last = -1;
    for (int x = 0; x < edges.cols; ++x) {
      if (edges.at<uint8_t>(y, x)) last = x;
      dx.at<int32_t>(y, x) = last < 0 ? INT32_MAX : x - last;
    }
    last = -1;
    for (int x = edges.cols - 1; x >= 0; --x) {
      if (edges.at<uint8_t>(y, x)) last = x;
      if (last >= 0) dx.at<int32_t>(y, x) =
          std::min(dx.at<int32_t>(y, x), last - x);
    }
  }
  return dx;
}

// Squared distance at row y from parabola centered at row v.
inline int64_t value(int v, int y, int64_t horizontal_sq) {
  int64_t delta = static_cast<int64_t>(y) - v;
  return horizontal_sq + delta * delta;
}

// Step 2: lower envelope of finite parabolas for each column.
cv::Mat exact_squared_distance(const cv::Mat& dx) {
  cv::Mat out(dx.size(), CV_32F);
  const int h = dx.rows;
  std::vector<int> sites(h);
  std::vector<int> boundaries(h + 1);
  std::vector<int64_t> f(h);
  for (int x = 0; x < dx.cols; ++x) {
    for (int y = 0; y < h; ++y) {
      int32_t d = dx.at<int32_t>(y, x);
      f[y] = d == INT32_MAX ? INF : static_cast<int64_t>(d) * d;
    }
    int k = -1;
    for (int q = 0; q < h; ++q) {
      if (f[q] == INF) continue;
      int start = 0;
      while (k >= 0) {
        const int p = sites[k];
        // First integer y where parabola q is strictly better than p.
        // value(q,y) < value(p,y) iff 2*y*(q-p) > f[q]+q*q-f[p]-p*p.
        int64_t num = f[q] + static_cast<int64_t>(q)*q
                    - f[p] - static_cast<int64_t>(p)*p;
        int64_t den = 2LL * (q - p);
        // floor(num / den) for possibly negative numerator.
        int64_t floor_div = num >= 0 ? num / den : -((-num + den - 1) / den);
        int64_t cross = floor_div + 1;
        if (cross <= boundaries[k]) {
          --k;
        } else {
          start = static_cast<int>(std::min<int64_t>(h, std::max<int64_t>(0, cross)));
          break;
        }
      }
      if (k < 0) start = 0;
      if (start >= h) continue;
      ++k;
      sites[k] = q;
      boundaries[k] = start;
    }
    if (k < 0) {
      for (int y = 0; y < h; ++y) out.at<float>(y,x) = 0.0f;
      continue;
    }
    int active = 0;
    for (int y = 0; y < h; ++y) {
      while (active < k && boundaries[active + 1] <= y) ++active;
      int v = sites[active];
      out.at<float>(y,x) = static_cast<float>(value(v, y, f[v]));
    }
  }
  return out;
}

cv::Mat formulate(cv::Mat d, const std::string& mode) {
  if (mode == "linear-bound") {
    cv::threshold(d, d, 6.0, 6.0, cv::THRESH_TRUNC);
  } else if (mode == "logarithmic") {
    cv::log(d + 1.0f, d);
  } else if (mode == "exponential") {
    cv::exp(-d / (6.0f / 5.541f), d);
    d = 1.0f - d;
  } else if (mode != "linear") {
    throw std::runtime_error("Invalid formulation: " + mode);
  }
  cv::Mat result;
  cv::normalize(d, result, 0, 255, cv::NORM_MINMAX, CV_8U);
  return result;
}

cv::Mat original_method(const cv::Mat& edges, const std::string& mode) {
  cv::Mat squared = exact_squared_distance(horizontal_distances(edges));
  cv::sqrt(squared, squared);
  return formulate(squared, mode);
}

cv::Mat opencv_reference(const cv::Mat& edges, const std::string& mode) {
  cv::Mat mask, inv, d;
  cv::threshold(edges, mask, 0, 255, cv::THRESH_BINARY);
  cv::bitwise_not(mask, inv);
  cv::distanceTransform(inv, d, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
  return formulate(d, mode);
}
}

class OriginalDistanceNode : public rclcpp::Node {
public:
  OriginalDistanceNode() : Node("distance_surface_original_node") {
    mode_ = declare_parameter<std::string>(
      "distance_surface_formulation", "exponential");

    processing_mode_ = declare_parameter<std::string>(
      "distance_processing_mode", "cpu");

    if (processing_mode_ != "cpu" && processing_mode_ != "gpu")
      throw std::runtime_error(
        "distance_processing_mode must be cpu or gpu");
    if (mode_ != "linear" && mode_ != "linear-bound" &&
        mode_ != "logarithmic" && mode_ != "exponential")
      throw std::runtime_error("Invalid formulation");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    pub_ = create_publisher<sensor_msgs::msg::Image>("distance_surface_original", qos);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      "denoised_filled_edge_image", qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
          cv::Mat edges = cv_bridge::toCvCopy(msg, "mono8")->image;
          if (edges.empty()) return;
          ++received_;
          if (cv::countNonZero(edges) == 0) {
            RCLCPP_WARN(get_logger(), "Input frame %zu has no edges; skipping (compatible behavior)", received_);
            return;
          }
          cv::Mat result;

          if (processing_mode_ == "gpu") {
            result = distance_surface_gpu(edges, mode_);
          } else {
            result = original_method(edges, mode_);
          }

          pub_->publish(
            *cv_bridge::CvImage(
              msg->header, "mono8", result).toImageMsg());

          if (processing_mode_ == "cpu") {
            cv::Mat reference = opencv_reference(edges, mode_);
            cv::Mat difference;
            cv::absdiff(result, reference, difference);

            double max_diff = 0;
            cv::minMaxLoc(
              difference, nullptr, &max_diff);

            RCLCPP_INFO(
              get_logger(),
              "Frame %zu: mode=cpu, mismatch_pixels=%d, max_diff=%.0f, nonzero=%d",
              received_,
              cv::countNonZero(difference),
              max_diff,
              cv::countNonZero(result));
          } else {
            RCLCPP_INFO(
              get_logger(),
              "Frame %zu: mode=gpu, %dx%d, nonzero=%d",
              received_,
              result.cols,
              result.rows,
              cv::countNonZero(result));
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Distance computation failed: %s", e.what());
        }
      });
    RCLCPP_INFO(get_logger(), "Distance node ready; mode=%s formulation=%s",
      processing_mode_.c_str(), mode_.c_str());
  }
private:
  std::string mode_;
  std::string processing_mode_;
  size_t received_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<OriginalDistanceNode>()); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("distance_surface_original_node"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
