#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

// A self-contained stage-1 adapter. TXT: timestamp_seconds x y polarity.
// It intentionally does NOT pretend to read Prophesee ROS1 bags.
struct Event { double t; int x; int y; bool polarity; };
class EventsTxtNode : public rclcpp::Node {
public:
  EventsTxtNode() : Node("events_txt_node") {
    auto file = declare_parameter<std::string>("input_file", "");
    width_ = declare_parameter<int>("width", 240);
    height_ = declare_parameter<int>("height", 180);
    window_ms_ = declare_parameter<int>("accumulation_window_ms", 25);
    startup_delay_ms_ = declare_parameter<int>("startup_delay_ms", 2000);
    denoise_ = declare_parameter<int>("denoising_min_neighbours", 1);
    fill_ = declare_parameter<int>("filling_min_neighbours", 4);
    if(width_ <= 0 || height_ <= 0 || window_ms_ <= 0) throw std::runtime_error("Invalid dimensions or window");
    if(file.empty()) throw std::runtime_error("Set input_file to a TXT file");
    std::ifstream in(file);
    if(!in) throw std::runtime_error("Cannot open input_file: " + file);
    std::string line;
    while(std::getline(in,line)) {
      if(line.empty() || line[0]=='#') continue;
      std::istringstream s(line);
      double t; int x,y,p;
      if(s>>t>>x>>y>>p && std::isfinite(t)) events_.push_back({t,x,y,p!=0});
    }
    if(events_.empty()) throw std::runtime_error("No parseable events");
    if(!std::is_sorted(events_.begin(),events_.end(),[](const Event&a,const Event&b){return a.t<b.t;}))
      std::stable_sort(events_.begin(),events_.end(),[](const Event&a,const Event&b){return a.t<b.t;});
    pub_edges_=create_publisher<sensor_msgs::msg::Image>("edge_image",10);
    pub_filtered_=create_publisher<sensor_msgs::msg::Image>("denoised_filled_edge_image",10);
    if (startup_delay_ms_ < 0) throw std::runtime_error("startup_delay_ms must be nonnegative");
    timer_=create_wall_timer(std::chrono::milliseconds(window_ms_),[this](){tick();});
    ready_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(startup_delay_ms_);
    RCLCPP_INFO(get_logger(),"Loaded %zu events; %dx%d; window %d ms",events_.size(),width_,height_,window_ms_);
  }
private:
  static cv::Mat filter(const cv::Mat& src,int nd,int nf){
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
  void tick(){
    if (std::chrono::steady_clock::now() < ready_at_) return;
    if(cursor_>=events_.size()){
      RCLCPP_INFO(get_logger(),"Finished TXT replay (%zu events)",events_.size());
      timer_->cancel(); return;
    }
    cv::Mat edges=cv::Mat::zeros(height_,width_,CV_8UC1);
    const double start=events_[cursor_].t, end=start+window_ms_/1000.0;
    while(cursor_<events_.size() && events_[cursor_].t<end){
      const auto &e=events_[cursor_++];
      if(e.x>=0&&e.x<width_&&e.y>=0&&e.y<height_) edges.at<uint8_t>(e.y,e.x)=255;
    }
    std_msgs::msg::Header header; header.stamp=now();
    auto edge_msg=cv_bridge::CvImage(header,"mono8",edges).toImageMsg();
    pub_edges_->publish(*edge_msg);
    RCLCPP_INFO(get_logger(),"Published edge frame %zu, nonzero=%d, subscribers=%zu",
                ++published_frames_, cv::countNonZero(edges), pub_edges_->get_subscription_count());
    auto filtered=filter(edges,denoise_,fill_);
    auto filtered_msg=cv_bridge::CvImage(header,"mono8",filtered).toImageMsg();
    pub_filtered_->publish(*filtered_msg);
  }
  int width_,height_,window_ms_,startup_delay_ms_,denoise_,fill_;
  size_t published_frames_=0;
  std::chrono::steady_clock::time_point ready_at_;
  std::vector<Event> events_; size_t cursor_=0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_edges_,pub_filtered_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  try{rclcpp::spin(std::make_shared<EventsTxtNode>());}
  catch(const std::exception&e){RCLCPP_ERROR(rclcpp::get_logger("events_txt_node"),"%s",e.what());rclcpp::shutdown();return 1;}
  rclcpp::shutdown(); return 0;
}
