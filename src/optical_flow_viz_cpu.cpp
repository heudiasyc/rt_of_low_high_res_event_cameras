/**
 * optical_flow_viz_cpu.cpp
 * This code corresponds to the computation of the optical flow visualization, on CPU.
 */

#include "rt_of_low_high_res_event_cameras/optical_flow_viz_cpu.hpp"


/**
 * \brief Computes the optical flow visualization as an image of the vector field represented as
 * arrows, using the CPU.
 * 
 * \param of_u The input flow along the x axis
 * \param of_v The input flow along the y axis
 * \param optical_flow_viz The destination image
 */
void optical_flow_viz_arrows_cpu(const Mat& of_u, const Mat& of_v, Mat& optical_flow_viz)
{
  // We begin by initializing the visualization image with black pixels
  optical_flow_viz = Scalar(0, 0, 0);
 
  // For every few pixels...
  for(int y = 0; y < of_u.rows; y+=VIZ_ARROWS_SPREAD) {
    for(int x = 0; x < of_u.cols; x+=VIZ_ARROWS_SPREAD) {
      // ... if there is some optical flow, and if it has a magnitude of at least 1px...
      if(pow(of_u.at<float>(y, x), 2) + pow(of_v.at<float>(y, x), 2) > 1) {
        // ... we display a white arrow, with an increased size to improve the visualization.
        arrowedLine(
          optical_flow_viz, Point(x, y), Point(
            x + VIZ_ARROWS_FACTOR*of_u.at<float>(y, x),
            y + VIZ_ARROWS_FACTOR*of_v.at<float>(y, x)),
          Scalar(255, 255, 255));
      }
    }
  }
}


/**
 * \brief Computes the optical flow visualization as a RGB image using a simple colorwheel, using
 * the CPU. This method is a fast version, based on a OpenCV tutorial:
 * https://docs.opencv.org/4.2.0/d4/dee/tutorial_optical_flow.html
 * 
 * \param of_u The input flow along the x axis
 * \param of_v The input flow along the y axis
 * \param optical_flow_viz The destination image
 * \param clip_flow Should the flow be scaled with a fixed upper bound? If so, clip_flow should
 * indicate this upper bound (a value <= 0 deactivates it)
 */
void optical_flow_viz_colors_cpu(
  const Mat& of_u, const Mat& of_v, Mat& optical_flow_viz, float clip_flow)
{
  // We begin by computing the magnitude and angle of the optical flow for each pixel
  Mat mag, ang;
  cartToPolar(of_u, of_v, mag, ang, true);

  // We then normalize the magnitudes to values between 0 and 1, while taking into account the
  // clip_flow value if needed
  Mat mag_normalized;
  if(clip_flow > 0) {
    threshold(mag, mag_normalized, clip_flow, 0, THRESH_TRUNC);
    mag_normalized /= clip_flow;
  } else {
    normalize(mag, mag_normalized, 0, 1, NORM_MINMAX);
  }

  // We use these magnitude and angle matrices to form an HSV image
  Mat hsv_split[3], hsv;
  hsv_split[0] = ang;
  hsv_split[1] = Mat::ones(of_u.size(), CV_32F);
  hsv_split[2] = mag_normalized;
  merge(hsv_split, 3, hsv);

  // We convert this HSV image back to a BGR image
  Mat optical_flow_viz_32;
  cvtColor(hsv, optical_flow_viz_32, COLOR_HSV2BGR);

  // For a better visibility, we transform the pixels without flow (= black pixels) into gray ones
  for(int y = 0; y < optical_flow_viz_32.rows; ++y) {
    for(int x = 0; x < optical_flow_viz_32.cols; ++x) {
      if(optical_flow_viz_32.at<Vec3f>(y, x) == Vec3f(0, 0, 0)) {
        optical_flow_viz_32.at<Vec3f>(y, x) = Vec3f(.5, .5, .5);
      }
    }
  }

  // And finally, we convert the float type image to a uint8_t image
  optical_flow_viz_32.convertTo(optical_flow_viz, CV_8UC3, 255);
}


/**
 * \brief Computes the optical flow visualization, according to the wanted method, using the CPU.
 * 
 * \param optical_flow The input flow matrix
 * \param viz_method The visualization method (either "colors" or "arrows")
 * \param clip_flow This parameter is only useful if the "colors" method has been selected (it will
 * simply not be considered for the "arrows" method). If its value is > 0, then this value will be
 * used as an upper bound for the flow magnitudes: every value above it will be brought back to
 * this value, and the normalization of the flow magnitudes will always be between 0 and this
 * value. This setting helps having a constant scale for the optical flow visualization, but its
 * value should be carefully selected to avoid obtaining strange visual results.
 */
Mat optical_flow_viz_cpu(const Mat& optical_flow, const string& viz_method, float clip_flow)
{
  // We first have to split the 2-channel optical flow matrix into two distinct matrices, one
  // holding the flow on the x axis (u), and the other one holding the flow on the y axis (v) 
  vector<Mat> u_v_array;
  u_v_array.reserve(2);
  split(optical_flow, u_v_array);
  Mat u = u_v_array[0], v = u_v_array[1];

  // We then initialize an empty matrix, that will in the end be the flow visualization image
  Mat optical_flow_viz(optical_flow.size(), CV_8UC3);

  // Then, depending on the chosen visualization method, the visualization image is created
  if(viz_method == "colors") {
    optical_flow_viz_colors_cpu(u, v, optical_flow_viz, clip_flow);
  } else if(viz_method == "arrows") {
    optical_flow_viz_arrows_cpu(u, v, optical_flow_viz);
  } else {
    ROS_ERROR_STREAM("Unknown flow visualization choice '" << viz_method << "', exiting!");
    exit(EXIT_FAILURE);
  }

  // Finally, we return the generated visualization
  return optical_flow_viz;
}
