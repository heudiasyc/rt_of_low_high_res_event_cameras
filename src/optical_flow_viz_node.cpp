/**
 * optical_flow_viz_node.cpp
 * This code corresponds to the ROS node which goal is to produce a nice visualization of the
 * optical flow results as a RGB image using a predefined colorwheel, or as an image representing
 * the flow vector field as arrows.
 * This is the fifth (and final) node of the pipeline architecture.
 */

#include "rt_of_low_high_res_event_cameras/defines.hpp"
#include "rt_of_low_high_res_event_cameras/includes.hpp"

#include "rt_of_low_high_res_event_cameras/optical_flow_viz_cpu.hpp"


/**
 * Global variables definition
 */

// ROS publisher, used to send the optical flow images once computed
ros::Publisher optical_flow_viz_pub;

// Optical flow visualization to use (arrows or colors)
string visualization_method;

// Upper bound for the optical flow visualization, if the colorwheel-based method has been chosen
float clip_flow;


/**
 * \brief ROS callback, called when a new optical flow matrix arrives.
 * From it, a human-readable visualization is computed, and the generated image is then sent for
 * display.
 */
void callback_optical_flow(const sensor_msgs::Image::ConstPtr& img)
{
  // Converting the message to an usable OpenCV Mat
  cv::Mat optical_flow;
  try {
    optical_flow = cv_bridge::toCvShare(img)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }
  
  // Computing the visualization from the optical flow matrix
  Mat optical_flow_viz = optical_flow_viz_cpu(optical_flow, visualization_method, clip_flow);

  // Converting the optical_flow_viz matrix to a ROS compatible msg and publishing it
  cv_bridge::CvImage optical_flow_viz_msg;
  optical_flow_viz_msg.encoding = "bgr8";
  optical_flow_viz_msg.image = optical_flow_viz;
  optical_flow_viz_pub.publish(optical_flow_viz_msg.toImageMsg());
}


/**
 * \brief Main function.
 */
int main(int argc, char* argv[])
{
  // ROS node initialization
  ros::init(argc, argv, "optical_flow_node");
  ros::NodeHandle n("~");
  string ns = "/rt_of_low_high_res_event_cameras";

  // Configuration of the visualization setting (arrows or colors)
  n.param<string>("visualization_method", visualization_method, VIZ_DEFAULT);

  // Collection of the flow clipping value (facultative, and only for the colors visualization)
  n.param<float>("clip_flow", clip_flow, CLIP_FLOW_DEFAULT);

  // Collection of the subscriber/publisher queues size
  int sub_pub_queues_size;
  n.param<int>("queues_size", sub_pub_queues_size, QUEUES_SIZE_DEFAULT);

  // Configuration of the optical flow subscriber and visualization publisher
  ros::Subscriber optical_flow_sub = n.subscribe(
    ns+"/optical_flow", sub_pub_queues_size, callback_optical_flow);
  optical_flow_viz_pub = n.advertise<sensor_msgs::Image>(
    ns+"/optical_flow_viz", sub_pub_queues_size);

  // Call to ros::spin() to process incomming events until Ctrl+C is pressed
  ros::spin();

  return 0;
}
