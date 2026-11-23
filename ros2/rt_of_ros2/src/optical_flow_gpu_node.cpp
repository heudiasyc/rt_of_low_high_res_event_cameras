#include <cuda_runtime.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <flowfilter/gpu/flowfilter.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

static void wrap(cv::Mat &m, flowfilter::image_t &i) {
  i.height=m.rows; i.width=m.cols; i.depth=m.channels();
  i.pitch=m.step; i.itemSize=m.elemSize1(); i.data=m.ptr();
}

class GpuFlowNode : public rclcpp::Node {
public:
  GpuFlowNode() : Node("optical_flow_gpu_node") {
    levels_=declare_parameter<int>("pyramidal_flow_levels",3);
    max_flow_=declare_parameter<double>("max_flow",15.0);
    gammas_=declare_parameter<std::vector<double>>("gammas",{500.,500.,500.});
    iterations_=declare_parameter<std::vector<int64_t>>("smooth_iterations",{20,20,20});
    if(levels_<1 || levels_>10 || max_flow_<=0 ||
       static_cast<int>(gammas_.size())!=levels_ ||
       static_cast<int>(iterations_.size())!=levels_) {
      throw std::runtime_error("Invalid flow parameters or vector lengths");
    }
    auto qos=rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    pub_=create_publisher<sensor_msgs::msg::Image>("/optical_flow",qos);
    sub_=create_subscription<sensor_msgs::msg::Image>("/distance_surface",qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m){process(m);});
    RCLCPP_INFO(get_logger(),"GPU flow node ready, levels=%d",levels_);
  }
private:
  void process(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    auto callback_start = std::chrono::steady_clock::now();
    try {
      if(msg->encoding!="mono8") {
        RCLCPP_ERROR(get_logger(),"Expected mono8, got %s",msg->encoding.c_str()); return;
      }
      auto cv_ptr = cv_bridge::toCvShare(msg, "mono8");
    cv::Mat input = cv_ptr->image;
      if(input.empty()) return;
      if(!input.isContinuous()) input=input.clone();
      if(!filter_ || input.rows!=rows_ || input.cols!=cols_) {
        rows_=input.rows; cols_=input.cols;
        filter_=std::make_unique<flowfilter::gpu::PyramidalFlowFilter>(rows_,cols_,levels_);
        filter_->setMaxFlow(static_cast<float>(max_flow_));
        std::vector<float> gs; for(double x:gammas_) gs.push_back(static_cast<float>(x));
        std::vector<int> its; for(auto x:iterations_) its.push_back(static_cast<int>(x));
        filter_->setGamma(gs);
        filter_->setSmoothIterations(its);
        RCLCPP_INFO(get_logger(),"Initialized FlowFilter GPU %dx%d",cols_,rows_);
      }
      cv::Mat flow(input.rows,input.cols,CV_32FC2,cv::Scalar(0,0));
      flowfilter::image_t in{}; flowfilter::image_t out{};
      wrap(input,in); wrap(flow,out);
      auto perf_start = std::chrono::steady_clock::now();

      filter_->loadImage(in);
      filter_->compute();
      filter_->downloadFlow(out);

      cudaError_t err = cudaDeviceSynchronize();
      if (err != cudaSuccess) {
        throw std::runtime_error(
          std::string("CUDA error: ") + cudaGetErrorString(err));
      }

      auto perf_end = std::chrono::steady_clock::now();
      double perf_ms =
        std::chrono::duration<double, std::milli>(
          perf_end - perf_start).count();

      RCLCPP_INFO(
        get_logger(),
        "PERF_FLOW frame=%zu time_ms=%.6f",
        count_ + 1, perf_ms);

      cv::Mat masked=cv::Mat::zeros(flow.size(),CV_32FC2);
      flow.copyTo(masked,input==0);
      auto result=cv_bridge::CvImage(msg->header,"32FC2",masked).toImageMsg();
      pub_->publish(*result);
      ++count_;
    auto callback_end = std::chrono::steady_clock::now();

    static double sum_total_ms = 0.0;
    static double sum_gpu_ms = 0.0;
    static size_t samples = 0;

    double total_ms =
      std::chrono::duration<double, std::milli>(
        callback_end - callback_start).count();

    sum_total_ms += total_ms;
    sum_gpu_ms += perf_ms;
    ++samples;

    if(samples == 100) {
      RCLCPP_WARN(
        get_logger(),
        "PERF_AVG frame=%zu total_avg=%.3f gpu_avg=%.3f other_avg=%.3f",
        count_,
        sum_total_ms / samples,
        sum_gpu_ms / samples,
        (sum_total_ms - sum_gpu_ms) / samples);

      sum_total_ms = 0.0;
      sum_gpu_ms = 0.0;
      samples = 0;
    }


      RCLCPP_INFO(get_logger(),"Published GPU flow frame %zu, %dx%d",count_,cols_,rows_);
    } catch(const std::exception &e) {
      RCLCPP_ERROR(get_logger(),"GPU flow failed: %s",e.what());
    }
  }
  int levels_,rows_=0,cols_=0; double max_flow_;
  std::vector<double> gammas_; std::vector<int64_t> iterations_;
  size_t count_=0;
  std::unique_ptr<flowfilter::gpu::PyramidalFlowFilter> filter_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try {rclcpp::spin(std::make_shared<GpuFlowNode>());}
  catch(const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("gpu_flow"),"Fatal: %s",e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
