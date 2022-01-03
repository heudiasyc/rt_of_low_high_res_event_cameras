/**
 * optical_flow_node.cpp
 * This code corresponds to the ROS node which goal is to compute the optical flow from the
 * distance surface images.
 * This is the fourth node of the pipeline architecture.
 */

#include "rt_of_low_high_res_event_cameras/defines.hpp"
#include "rt_of_low_high_res_event_cameras/includes.hpp"

#include "rt_of_low_high_res_event_cameras/optical_flow_gpu.hpp"


/**
 * Global variables definition
 */

// ROS publisher, used to send the distance surface image once computed
ros::Publisher optical_flow_pub;

// Number of pyramidal levels for the optical flow library
int pyramidal_flow_levels;

// Number of pyramidal levels for the optical flow library
float max_flow;

// Regularization parameters for each layer of the pyramidal optical flow computation module
vector<float> gammas;

// Number of smoothing iterations for each layer of the pyramidal optical flow computation module
vector<int> smooth_iterations;


/**
 * \brief ROS callback, called when a new distance surface image arrives.
 * From this image, its optical flow is computed using the flowfilter library, and then sent for
 * further operations with it.
 */
void callback_dist_surface(const sensor_msgs::Image::ConstPtr& img)
{
  // Converting the message to an usable OpenCV Mat
  Mat dist_surface;
  try {
    dist_surface = cv_bridge::toCvShare(img)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  // Computing the optical flow from the distance surface image, using the flowfilter GPU library
  Mat optical_flow = optical_flow_gpu(
    dist_surface, pyramidal_flow_levels, max_flow, gammas, smooth_iterations);

  // Converting the optical_flow Mat to a ROS compatible msg and publishing it
  cv_bridge::CvImage optical_flow_msg;
  optical_flow_msg.encoding = "32FC2";
  optical_flow_msg.image = optical_flow;
  optical_flow_pub.publish(optical_flow_msg.toImageMsg());
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

  // Collection of the number of pyramidal flow levels
  n.param<int>("pyramidal_flow_levels", pyramidal_flow_levels, PYRAMIDAL_FLOW_LEVELS_DEFAULT);

  // Collection of the maximum optical flow magnitude
  n.param<float>("max_flow", max_flow, MAX_FLOW_DEFAULT);

  // Collection of the gamma values for the optical flow config
  n.param("gammas", gammas, GAMMAS_DEFAULT);

  // Collection of the smooth iterations for the optical flow config
  n.param("smooth_iterations", smooth_iterations, SMOOTH_ITERATIONS_DEFAULT);

  // Collection of the subscriber/publisher queues size
  int sub_pub_queues_size;
  n.param<int>("queues_size", sub_pub_queues_size, QUEUES_SIZE_DEFAULT);

  // Configuration of the distance surface subscriber and the optical flow publisher
  ros::Subscriber sub_dist_surface = n.subscribe(
    ns+"/distance_surface", sub_pub_queues_size, callback_dist_surface);
  optical_flow_pub = n.advertise<sensor_msgs::Image>(ns+"/optical_flow", sub_pub_queues_size);

  // Call to ros::spin() to process incomming events until Ctrl+C is pressed
  ros::spin();

  return 0;
}
