/**
 * optical_flow_gpu.hpp
 * Header file for the computation of the optical flow from the distance surface images, on GPU.
 */

#pragma once

#include "rt_of_low_high_res_event_cameras/includes.hpp"


/**
 * \brief Computes optical flow from the given distance surface image, using the flowfilter GPU
 * library, and applies the adequate mask to only return flow results for the pixels that have
 * received at least an event.
 *
 * \param dist_surface The distance surface image
 * \param pyramidal_levels The number of pyramidal levels the optical flow flowfilter library
 * should be configured with
 * \param max_flow The maximum optical flow (in pixels) the flowfilter library should be configured
 * with
 * \param gamma The regularization weights for each pyramidal level of the flowfilter library
 * \param smooth_iterations The regularization weights for each pyramidal level of the flowfilter
 * library
 *
 * \return The optical flow results as an OpenCV matrix, where only the edge pixels have a value
 */
Mat optical_flow_gpu(
  Mat& dist_surface, int pyramidal_levels, float max_flow, const vector<float>& gamma,
  const vector<int>& smooth_iterations);
