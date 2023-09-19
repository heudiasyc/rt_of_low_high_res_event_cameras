/**
 * edges_cpu.cpp
 * This code corresponds to the events -> edge image computation, on CPU.
 * Note: no GPU alternative of this code was written, as little to no improvement on the
 * computation time would probably be achieved.
 */

#include "rt_of_low_high_res_event_cameras/edges_cpu.hpp"


/**
 * \brief Edge image creation function, which should be called at the end of the accumulation
 * window Δt. It consumes the events from the given buffer to create the edge image. Once done, the
 * created image is sent for further processing.
 *
 * \param evts_to_process Buffer of events to use for creating the edge image
 * \param edges_image_ptr Pointer to the output edge image OpenCV matrix 
 * \param height Height of the edge image
 * \param width Width of the edge image
 * \param undistortion_matrix Matrix for undistorting the events (if needed)
 */
void evts_to_edge_image(
  const vector<dvs_msgs::Event>& evts_to_process, Mat* edge_image_ptr, int height, int width,
  const Mat& undistortion_matrix)
{
  // If there is no event, we simply exit the function before doing anything
  if(evts_to_process.empty()) {
    return;
  }

  // We first initialize the edges image matrix...
  *edge_image_ptr = Mat::zeros(height, width, CV_8U);

  // ... and treat all the received events
  for(const dvs_msgs::Event& evt : evts_to_process) {
    // Reading the infos from the event
    int evt_x = evt.x, evt_y = evt.y;

    // Applying the distortion if needed
    if(!undistortion_matrix.empty()) {
      Point2i new_coordinates = undistortion_matrix.at<Point2i>(evt_y, evt_x);
      evt_x = new_coordinates.x;
      evt_y = new_coordinates.y;

      // Checking if the point is in the bounds after undistorting
      if(evt_x < 0 || evt_x >= width || evt_y < 0 || evt_y >= height) {
        continue;
      }
    }

    // Adding the event to the image
    edge_image_ptr->at<uint8_t>(evt_y, evt_x) = 255;
  }
}
