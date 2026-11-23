#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <cstdint>
#include <memory>

struct StreamEvent { double t; int x; int y; bool polarity; };

class EventsTxtStreamNode : public rclcpp::Node {
public:
  EventsTxtStreamNode() : Node("events_txt_stream_node") {
    publish_filtered_image_ = declare_parameter<bool>("publish_filtered_image", true);
    auto file = declare_parameter<std::string>("input_file", "");
    width_ = declare_parameter<int>("width", 240);
    height_ = declare_parameter<int>("height", 180);
    window_ms_ = declare_parameter<int>("accumulation_window_ms", 25);
    startup_delay_ms_ = declare_parameter<int>("startup_delay_ms", 2000);
    denoise_ = declare_parameter<int>("denoising_min_neighbours", 1);
    fill_ = declare_parameter<int>("filling_min_neighbours", 4);
    replay_interval_ms_ = declare_parameter<int>("replay_interval_ms", 100);
    if (width_<=0 || height_<=0 || window_ms_<=0 || startup_delay_ms_<0 || replay_interval_ms_<=0)
      throw std::runtime_error("Invalid dimensions/window/delay/interval");
    if (file.empty()) throw std::runtime_error("Set input_file");
    in_.open(file);
    if (!in_) throw std::runtime_error("Cannot open input_file: " + file);
    pub_edges_ = create_publisher<sensor_msgs::msg::Image>("edge_image", 10);
    pub_filtered_ = create_publisher<sensor_msgs::msg::Image>("denoised_filled_edge_image", 10);
    if (!read_next()) throw std::runtime_error("No parseable events");
    ready_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(startup_delay_ms_);
    timer_ = create_wall_timer(std::chrono::milliseconds(replay_interval_ms_), [this]{ tick(); });
    RCLCPP_INFO(get_logger(), "Streaming TXT: %dx%d, window=%d ms, replay_interval=%d ms", width_,height_,window_ms_,replay_interval_ms_);
  }
private:
  bool publish_filtered_image_ = true;
  bool read_next() {
    std::string line;
    while (std::getline(in_, line)) {
      ++lines_;
      if (line.empty() || line[0]=='#') continue;
      std::istringstream s(line);
      double t; int x,y,p;
      if (!(s>>t>>x>>y>>p) || !std::isfinite(t)) { ++invalid_lines_; continue; }
      if (has_previous_ && t < previous_t_) {
        throw std::runtime_error("Input timestamps not sorted at line " + std::to_string(lines_) + "; old node sorts these in memory");
      }
      previous_t_=t;
      has_previous_=true;
      next_ = StreamEvent{t,x,y,p!=0};
      has_next_=true;
      return true;
    }
    has_next_=false;
    return false;
  }
  static cv::Mat filter(const cv::Mat& src, int nd, int nf) {
    cv::Mat d=src.clone(),f;
    if(nd>=1&&nd<=4) for(int y=1;y<src.rows-1;++y) for(int x=1;x<src.cols-1;++x){
      if(!src.at<uint8_t>(y,x))continue;
      int n=(src.at<uint8_t>(y-1,x)!=0)+(src.at<uint8_t>(y+1,x)!=0)+
            (src.at<uint8_t>(y,x-1)!=0)+(src.at<uint8_t>(y,x+1)!=0);
      if(n<nd)d.at<uint8_t>(y,x)=0;
    }
    f=d.clone();
    if(nf>=1&&nf<=4) for(int y=1;y<d.rows-1;++y) for(int x=1;x<d.cols-1;++x){
      if(d.at<uint8_t>(y,x))continue;
      int n=(d.at<uint8_t>(y-1,x)!=0)+(d.at<uint8_t>(y+1,x)!=0)+
            (d.at<uint8_t>(y,x-1)!=0)+(d.at<uint8_t>(y,x+1)!=0);
      if(n>=nf)f.at<uint8_t>(y,x)=255;
    }
    return f;
  }
  void tick() {
    if(std::chrono::steady_clock::now() < ready_at_) return;
    if(!has_next_) { finish(); return; }
    cv::Mat edges=cv::Mat::zeros(height_,width_,CV_8UC1);
    const double start=next_.t, end=start+window_ms_/1000.0;
    uint64_t frame_events=0;
    do {
      const StreamEvent e=next_;
      ++frame_events;
      ++processed_events_;
      if(e.x>=0&&e.x<width_&&e.y>=0&&e.y<height_) edges.at<uint8_t>(e.y,e.x)=255;
      if(!read_next()) break;
    } while(has_next_ && next_.t < end);
    std_msgs::msg::Header header;
    header.stamp=now();
    auto edge_msg=cv_bridge::CvImage(header,"mono8",edges).toImageMsg();
    pub_edges_->publish(*edge_msg);
    auto filtered=filter(edges,denoise_,fill_);
    auto filtered_msg=cv_bridge::CvImage(header,"mono8",filtered).toImageMsg();
    if (publish_filtered_image_) pub_filtered_->publish(*filtered_msg);
    ++published_frames_;
    RCLCPP_INFO(get_logger(),"Frame %zu: t=%.6f, events=%llu, edge_nonzero=%d, filtered_nonzero=%d",
      published_frames_,start,static_cast<unsigned long long>(frame_events),
      cv::countNonZero(edges),cv::countNonZero(filtered));
    if(!has_next_) finish();
  }
  void finish() {
    if(timer_) timer_->cancel();
    RCLCPP_INFO(get_logger(),"Finished streaming: frames=%zu events=%llu invalid_lines=%llu",
      published_frames_,static_cast<unsigned long long>(processed_events_),static_cast<unsigned long long>(invalid_lines_));
  }
  int width_,height_,window_ms_,startup_delay_ms_,denoise_,fill_,replay_interval_ms_;
  size_t published_frames_=0;
  uint64_t lines_=0, invalid_lines_=0, processed_events_=0;
  double previous_t_=0.0;
  bool has_previous_=false, has_next_=false;
  StreamEvent next_{};
  std::ifstream in_;
  std::chrono::steady_clock::time_point ready_at_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_edges_,pub_filtered_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char**argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<EventsTxtStreamNode>()); }
  catch(const std::exception &e) { RCLCPP_ERROR(rclcpp::get_logger("events_txt_stream_node"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
