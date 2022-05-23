/**
 * utils.cpp
 * Collection of utility functions, used in various places in the code.
 */

#include "rt_of_low_high_res_event_cameras/utils.hpp"

#include <opencv2/calib3d/calib3d.hpp>


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
Mat init_undistort_mat(const string& filename, const int height, const int width)
{
  // Opening the calibration file
  FileStorage fs(filename, FileStorage::READ);
  if(!fs.isOpened()) {
    ROS_ERROR_STREAM("Couldn't open the specified camera parameters file: " << filename);
    exit(EXIT_FAILURE);
  }

  // Reading the camera matrix and distortion coefficients from it
  Mat camera_matrix, dist_coeffs;
  fs["camera_matrix"] >> camera_matrix;
  fs["distortion_coefficients"] >> dist_coeffs;

  // Creating a dummy array containing all points from the image
  vector<Point2f> points_to_undistort;
  for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      points_to_undistort.push_back(Point2f(x, y));
    }
  }

  // Undistorting all the points
  vector<Point2f> undistorted_pts;
  undistortPoints(
    points_to_undistort, undistorted_pts, camera_matrix, dist_coeffs, noArray(), camera_matrix);

  // Sorting them in a matrix for better manipulation further on
  Mat undistortion_mat(height, width, CV_32SC2);
  for(int y = 0; y < height; ++y) {
    for(int x = 0; x < width; ++x) {
      undistortion_mat.at<Point2i>(y, x) = undistorted_pts.at(y*width+x);
    }
  }

  return undistortion_mat;
}
