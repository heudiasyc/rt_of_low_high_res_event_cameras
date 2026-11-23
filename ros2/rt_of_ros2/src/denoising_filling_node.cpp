#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <cstdint>

cv::Mat denoising_filling_gpu(const cv::Mat&, int, int);
void free_gpu_memory();

static cv::Mat denoising_filling_cpu(const cv::Mat& edges, int nd, int nf) {
  cv::Mat denoised = edges.clone();
  if (nd > 0 && nd <= 4) {
    for (int y=1; y<edges.rows-1; ++y) for (int x=1; x<edges.cols-1; ++x) {
      if (!edges.at<uint8_t>(y,x)) continue;
      const int n = (edges.at<uint8_t>(y-1,x)!=0) + (edges.at<uint8_t>(y+1,x)!=0)
                  + (edges.at<uint8_t>(y,x-1)!=0) + (edges.at<uint8_t>(y,x+1)!=0);
      if (n < nd) denoised.at<uint8_t>(y,x) = 0;
    }
  }
  cv::Mat filled = denoised.clone();
  if (nf > 0 && nf <= 4) {
    for (int y=1; y<denoised.rows-1; ++y) for (int x=1; x<denoised.cols-1; ++x) {
      if (denoised.at<uint8_t>(y,x)) continue;
      const int n = (denoised.at<uint8_t>(y-1,x)!=0) + (denoised.at<uint8_t>(y+1,x)!=0)
                  + (denoised.at<uint8_t>(y,x-1)!=0) + (denoised.at<uint8_t>(y,x+1)!=0);
      if (n >= nf) filled.at<uint8_t>(y,x) = 255;
    }
  }
  return filled;
}

class DenoisingFillingNode : public rclcpp::Node {
public:
  DenoisingFillingNode() : Node("denoising_filling_node") {
    nd_ = declare_parameter<int>("denoising_min_neighbours", 2);
    nf_ = declare_parameter<int>("filling_min_neighbours", 3);
    mode_ = declare_parameter<std::string>("processing_mode", "cpu");
    if (mode_ != "cpu" && mode_ != "gpu")
      throw std::runtime_error("processing_mode must be cpu or gpu");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    pub_ = create_publisher<sensor_msgs::msg::Image>("denoised_filled_edge_image", qos);
    sub_ = create_subscription<sensor_msgs::msg::Image>("edge_image", qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
          cv::Mat edges = cv_bridge::toCvCopy(msg, "mono8")->image;
          if (edges.empty()) return;
          cv::Mat result = (mode_ == "gpu") ? denoising_filling_gpu(edges, nd_, nf_) : denoising_filling_cpu(edges, nd_, nf_);
          pub_->publish(*cv_bridge::CvImage(msg->header, "mono8", result).toImageMsg());
          RCLCPP_INFO(get_logger(), "Frame %zu: edge_nonzero=%d filtered_nonzero=%d",
                      ++count_, cv::countNonZero(edges), cv::countNonZero(result));
        } catch (const std::exception& e) {
          RCLCPP_ERROR(get_logger(), "Denoising/filling failed: %s", e.what());
        }
      });
    RCLCPP_INFO(get_logger(), "Denoising/filling ready, mode=%s nd=%d nf=%d", mode_.c_str(), nd_, nf_);
  }
private:
  std::string mode_;
  int nd_, nf_;
  size_t count_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<DenoisingFillingNode>()); }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("denoising_filling_node"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
