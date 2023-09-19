/**
 * optical_flow_gpu.cpp
 * This code corresponds to the computation of the optical flow from the distance surface images,
 * on GPU.
 */

#include "rt_of_low_high_res_event_cameras/optical_flow_gpu.hpp"

#include <flowfilter/gpu/flowfilter.h>


/**
 * Global variables definition
 */

// Pyramidal optical flow computation module, from the flowfilter library
flowfilter::gpu::PyramidalFlowFilter filter;

// Distance surface image wrapper for the flowfilter library
flowfilter::image_t dist_surface_wrapper;

// Optical flow matrix wrapper for the flowfilter library
flowfilter::image_t optical_flow_wrapper;

// OpenCV matrix, that will hold the optical flow after it is computed
Mat optical_flow;

// OpenCV matrix, that will hold the optical flow restricted only to the pixels for which an event
// was received
Mat optical_flow_masked;


/**
 * \brief Adequately wraps an OpenCV Mat image into its flowfilter::image_t equivalent, so that it
 * can be used for optical flow computation.
 *
 * \param cv_matrix The OpenCV-formated matrix to wrap
 * \param img The flowfilter image_t wrapper to initialize
 */
void wrap_cv_matrix(Mat& cv_matrix, flowfilter::image_t& img)
{
  img.height = cv_matrix.rows;
  img.width = cv_matrix.cols;
  img.depth = cv_matrix.channels();
  img.pitch = cv_matrix.cols*cv_matrix.elemSize();
  img.itemSize = cv_matrix.elemSize1();
  img.data = cv_matrix.ptr();
}


/**
 * \brief Computes optical flow from the given distance surface image, using the flowfilter GPU
 * library (https://github.com/jadarve/optical-flow-filter), and applies the adequate mask to only
 * return flow results for the pixels that have received at least an event.
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
  const vector<int>& smooth_iterations)
{
  // We first have to check if the image wrappers have already been initialized
  if(dist_surface_wrapper.data) {
    // If so, we only change the data pointer of the distance surface wrapper to make it point to
    // the newly received distance surface image, and reinitialize the optical_flow_masked matrix
    dist_surface_wrapper.data = dist_surface.ptr();
    optical_flow_masked = 0;
  } else {
    // If not, we have to initialize correctly both the distance surface image wrapper, as well as
    // the optical flow wrapper, the optical_flow and optical_flow_masked matrices, and the
    // pyramidal filter, using the received distance surface image data
    wrap_cv_matrix(dist_surface, dist_surface_wrapper);
    optical_flow = Mat(dist_surface.rows, dist_surface.cols, CV_32FC2);
    wrap_cv_matrix(optical_flow, optical_flow_wrapper);

    optical_flow_masked = Mat::zeros(dist_surface.rows, dist_surface.cols, CV_32FC2);

    filter = flowfilter::gpu::PyramidalFlowFilter(
      dist_surface.rows, dist_surface.cols, pyramidal_levels);
    filter.setMaxFlow(max_flow);
    filter.setGamma(gamma);
    filter.setSmoothIterations(smooth_iterations);
  }

  // Once done, we transfer the distance surface image to the GPU and make it compute the updated
  // flow
  filter.loadImage(dist_surface_wrapper);
  filter.compute();

  // Once computed, we transfer the computed optical flow from the GPU
  filter.downloadFlow(optical_flow_wrapper);

  // Finally, we extract only the optical flow for the edge pixels
  optical_flow.copyTo(optical_flow_masked, dist_surface == 0);

  // And we return this matrix
  return optical_flow_masked;
}
