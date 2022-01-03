/**
 * distance_surface_node.cpp
 * This code corresponds to the ROS node which goal is to compute the distance surface image from
 * the edge images.
 * This is the third node of the pipeline architecture.
 */

#include "rt_of_low_high_res_event_cameras/defines.hpp"
#include "rt_of_low_high_res_event_cameras/includes.hpp"

#include "rt_of_low_high_res_event_cameras/distance_surface_cpu.hpp"
#include "rt_of_low_high_res_event_cameras/distance_surface_gpu.hpp"


/**
 * Global variables definition
 */

// The function that will be used for computing the distance surfaces (CPU/GPU)
function<Mat(Mat, string)> dist_surface_function;

// String, containing the distance surface formulation that should be used
string dist_surface_formulation;

// ROS publisher, used to send the distance surface image once computed
ros::Publisher dist_surface_pub;


/**
 * \brief ROS callback, called when a new edge image message arrives.
 * From this image, its distance transform is computed, using the wanted formulation, and then sent
 * for further operations with it.
 */
void callback_edges_image(const sensor_msgs::Image::ConstPtr& img)
{
  // Converting the message to an usable OpenCV Mat
  Mat edges;
  try {
    edges = cv_bridge::toCvShare(img)->image;
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  // Computing the distance surface, using the correct CPU or GPU function
  const Mat dist_surface = dist_surface_function(edges, dist_surface_formulation);

  // Before sending, if the edge image contained no edge pixel, the returned Mat is empty. So, if
  // we detect this is the case here, we just discard it.
  if(dist_surface.empty()) {
    return;
  }

  // Converting the OpenCV Mat to a ROS compatible msg and publishing it
  cv_bridge::CvImage dist_surface_msg;
  dist_surface_msg.encoding = "mono8";
  dist_surface_msg.image = dist_surface;
  dist_surface_pub.publish(dist_surface_msg.toImageMsg());
}


/**
 * \brief Main function.
 */
int main(int argc, char* argv[])
{
  // ROS node initialization
  ros::init(argc, argv, "distance_surface_node");
  ros::NodeHandle n("~");
  string ns = "/rt_of_low_high_res_event_cameras";

  // Collection of the CPU/GPU parameter
  bool use_gpu_version;
  n.param<bool>("use_gpu_version", use_gpu_version, USE_GPU_VERSION_DEFAULT);
  if(use_gpu_version) {
    dist_surface_function = distance_surface_gpu;
  } else {
    dist_surface_function = distance_surface_cpu;
  }

  // Collection of the distance surface formulation to use (linear / bound / logarithmic /
  // exponential)
  n.param<string>(
    "distance_surface_formulation", dist_surface_formulation,
    DISTANCE_SURFACE_FORMULATION_DEFAULT);

  // Collection of the subscriber/publisher queues size
  int sub_pub_queues_size;
  n.param<int>("queues_size", sub_pub_queues_size, QUEUES_SIZE_DEFAULT);

  // Configuration of the denoised & filled edge image subscriber and the distance surface
  // publisher
  ros::Subscriber sub_edges = n.subscribe(
    ns+"/denoised_filled_edge_image", sub_pub_queues_size, callback_edges_image);
  dist_surface_pub = n.advertise<sensor_msgs::Image>(ns+"/distance_surface", sub_pub_queues_size);

  // Call to ros::spin() to process incomming events until Ctrl+C is pressed
  ros::spin();

  // Freeing GPU memory before leaving if needed
  if(use_gpu_version) {
    free_gpu_memory();
  }

  return 0;
}
