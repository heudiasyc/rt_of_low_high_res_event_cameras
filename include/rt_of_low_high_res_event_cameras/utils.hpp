/**
 * utils.hpp
 * Header file for utility functions, used in various places in the code.
 */

#pragma once

#include "rt_of_low_high_res_event_cameras/includes.hpp"


/**
 * \brief Using the calibration file, initializes a matrix of the specified size, where each cell
 * holds the coordinates of the cell after undistortion.
 * 
 * \param filename The path to the calibration file
 * \param height The height of the image
 * \param width The width of the image
 *
 * \return The undistortion matrix
 */
Mat init_undistort_mat(const string& filename, const int height, const int width);
