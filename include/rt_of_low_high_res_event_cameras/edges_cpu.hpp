/**
 * edges_cpu.hpp
 * Header file for the events -> edge image computation, on CPU.
 */

#pragma once

#include "rt_of_low_high_res_event_cameras/includes.hpp"

#include <dvs_msgs/Event.h>


/**
 * \brief Edge image creation function, which should be called at the end of the accumulation
 * window Δt. It consumes the events from the given buffer to create the edge image. Once done, the
 * created image is sent for further processing.
 *
 * \param evts_to_process Buffer of events to use for creating the edge image
 * \param edge_image_ptr Pointer to the output edge image OpenCV matrix
 * \param height Height of the edge image
 * \param width Width of the edge image
 * \param undistortion_matrix Matrix for undistorting the events (if needed)
 */
void evts_to_edge_image(
  const vector<dvs_msgs::Event>& evts_to_process, Mat* edge_image_ptr, int height, int width,
  const Mat& undistortion_matrix);
