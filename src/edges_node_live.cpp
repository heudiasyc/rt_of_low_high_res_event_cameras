/**
 * edges_node_live.cpp
 * This code corresponds to the ROS node which goal is to receive the events from the camera in
 * real-time, and to compute the edge images from them.
 * This is the first node of the pipeline architecture.
 */

#include "rt_of_low_high_res_event_cameras/defines.hpp"
#include "rt_of_low_high_res_event_cameras/utils.hpp"

#include "rt_of_low_high_res_event_cameras/edges_cpu.hpp"

#include <dvs_msgs/EventArray.h>
#include <thread>


/**
 * Global variables definition
 */

// The buffers of events, which function constantly alternates: while one is used for storing
// incomming events, the other one serves for creating the edge image from the events that were
// received
array<vector<dvs_msgs::Event>, 2> evt_buffers;

// A boolean to switch between which of the two event buffers should be used
bool buffer_used = 0;

// A mutex, used to lock the event buffers
mutex evt_buf_mutex;

// Height and width of the event sensor
int height, width;

// Undistortion matrix, used if a calibration file of the camera is given
Mat undistortion_matrix;

// ROS publisher, used to send the edge image once computed
ros::Publisher edges_pub;


/**
 * \brief Events processing function, which is just a global timer to indicate the end of the
 * accumulation window Δt, and to call the events -> edge image function accordingly.
 * 
 * \param accumulation_window The time window Δt, during which events should be accumulated to form
 * the edge images (in ms)
 */
void accumulation_timing_thread(int accumulation_window)
{
  // This thread runs until we exit the node
  while(ros::ok()) {
    // Before processing events, we switch the current buffer used to store the received events.
    // This way, the newly received events will be stored in the other buffer safely.
    {
      lock_guard<mutex> lock(evt_buf_mutex);
      buffer_used = !buffer_used;
    }

    // We call the events to edge image function (in a new thread to respect the accumulation
    // window)
    Mat edge_image;
    thread evts_processing_thread(
      evts_to_edge_image, evt_buffers[!buffer_used], &edge_image, height, width,
      undistortion_matrix);

    // We sleep during the processing
    this_thread::sleep_for(chrono::milliseconds(accumulation_window));

    // Before looping and processing events again, we have to make sure the previous iteration is
    // finished (which should be the case almost all the time).
    evts_processing_thread.join();

    // We clean the queue of its (now processed) events
    evt_buffers[!buffer_used].clear();

    // And we publish the computed edge image (if not empty)
    if(!edge_image.empty()) {
      cv_bridge::CvImage edges_msg;
      edges_msg.encoding = "mono8";
      edges_msg.image = edge_image;
      edges_pub.publish(edges_msg.toImageMsg());
    }
  }
}


/**
 * \brief ROS callback, called when a new DVS EventArray message arrives.
 * The events it contains are simply stored in the correct buffer, to be processed later on by the
 * events processing thread.
 * 
 * \note The events from the message can be either DVS- or Prophesee-formatted: we don't care, the
 * messages have the same fields, so the same checksum, and ROS therefore considers them as the
 * same type of message
 */
void callback_events(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // We first lock the event buffers, to avoid any collision
  lock_guard<mutex> lock(evt_buf_mutex);

  // We then append the received events at the back of the current buffer
  evt_buffers[buffer_used].insert(
    evt_buffers[buffer_used].end(), begin(msg->events), end(msg->events));
}


/**
 * \brief Main function.
 */
int main(int argc, char* argv[])
{
  // ROS node initialization
  ros::init(argc, argv, "edges_node");
  ros::NodeHandle n("~");
  string ns = "/rt_of_low_high_res_event_cameras";

  // Collection of the input topic
  string topic;
  n.param<string>("topic", topic, "");
  if(topic.empty()) {
    ROS_ERROR("No input topic selected, exiting!");
    exit(EXIT_FAILURE);
  }

  // Collection of the duration of the accumulation window (Δt)
  int accumulation_window;
  n.param<int>("accumulation_window", accumulation_window, -1);
  if(accumulation_window <= 0) {
    ROS_ERROR("No accumulation window given, exiting!");
    exit(EXIT_FAILURE);
  }

  // Collection of the image height and width, passed as mandatory parameters
  n.param<int>("height", height, -1);
  n.param<int>("width", width, -1);
  if(height <= 0 || width <= 0) {
    ROS_ERROR("Invalid height/width parameters, exiting!");
    exit(EXIT_FAILURE);
  }

  // Collection of the subscriber/publisher queues size
  int queues_size;
  n.param<int>("queues_size", queues_size, QUEUES_SIZE_DEFAULT);

  // If given, collection of the calibration file, and preparation of the undistortion matrix
  string calibration_file = n.param<string>("calibration_file", "");
  if(!calibration_file.empty()) {
    undistortion_matrix = init_undistort_mat(calibration_file, height, width);
  }

  // Configuration of the events subscriber and the edge image publisher
  ros::Subscriber evts_sub = n.subscribe(topic, queues_size, callback_events);
  edges_pub = n.advertise<sensor_msgs::Image>(ns+"/edge_image", queues_size);

  // Launch of the events processing thread
  thread acc_timing_thread(accumulation_timing_thread, accumulation_window);

  // Call to ros::spin() to process incomming events until Ctrl+C is pressed
  ros::spin();

  // Before exiting, we make sure join the separate thread
  acc_timing_thread.join();

  return 0;
}
