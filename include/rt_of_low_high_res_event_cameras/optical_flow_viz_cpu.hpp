/**
 * optical_flow_viz_cpu.hpp
 * Header file for the computation of the optical flow visualization, on CPU.
 */

#pragma once

#include "rt_of_low_high_res_event_cameras/defines.hpp"
#include "rt_of_low_high_res_event_cameras/includes.hpp"


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
Mat optical_flow_viz_cpu(const Mat& optical_flow, const string& viz_method, float clip_flow = -1);
