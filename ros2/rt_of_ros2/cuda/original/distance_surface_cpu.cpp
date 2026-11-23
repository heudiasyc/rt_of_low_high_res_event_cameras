/**
 * distance_surface_cpu.cpp
 * This code corresponds to the computation of the distance surfaces from the edge images, on CPU.
 */

#include "rt_of_low_high_res_event_cameras/distance_surface_cpu.hpp"


/**
 * \brief Computes the parabola ordinate (the F^i_y(j) function described in the paper used in the
 * distance transform algorithm, which link is given in the comment of the distance_surface_cpu
 * function)
 *
 * \param map_x Map obtained after the first step of the distance transform algorithm (distance
 * transform for the rows only)
 * \param i Current column
 * \param y Row of origin of the parabola
 * \param j Row for which we want to compute the ordinate
 *
 * \return The parabola ordinate at the given row coordinates
 */
int32_t parab_ord(const Mat& map_x, int i, int y, int j)
{
  // If, at its origin (lowest point), the parabola has a value of +∞, then it will necessarly also
  // have a value of +∞ for any other point
  if(map_x.at<int32_t>(y, i) == INT32_MAX) {
    return INT32_MAX;
  }

  // If this is not the case, we compute the ordinate at the given coordinates
  return map_x.at<int32_t>(y, i)*map_x.at<int32_t>(y, i) + (j-y)*(j-y);
}


/**
 * \brief Computes the abscissa of the intersection of two consecutive parabolas (the Sep^i(u, v)
 * function described in the paper used in the distance transform algorithm, which link is given in
 * the comment of the distance_surface_cpu function)
 *
 * \param map_x Map obtained after the first step of the distance transform algorithm (distance
 * transform for the rows only)
 * \param i Current column
 * \param u The row (abscissa) of the first parabola
 * \param v The row (abscissa) of the second parabola
 *
 * \return The abscissa of intersection of the two parabolas
 */
int32_t parab_inter_abs(const Mat& map_x, int i, int u, int v)
{
  // If, at their origin, one of the two parabolas has a value of +∞, then they will never
  // intersect, and we return +∞ to signal that
  if(map_x.at<int32_t>(u, i) == INT32_MAX || map_x.at<int32_t>(v, i) == INT32_MAX) {
    return INT32_MAX;
  }

  // Otherwise, we compute the abscissa of intersection of the two parabolas
  return (v*v - u*u + map_x.at<int32_t>(v, i)*map_x.at<int32_t>(v, i)
    - map_x.at<int32_t>(u, i)*map_x.at<int32_t>(u, i)) / (2*(v-u));
}


/**
 * \brief Computes the distance surface from an edge image, on CPU, using the exact method
 * described in
 * https://pageperso.lif.univ-mrs.fr/~edouard.thiel/print/2007-geodis-thiel-coeurjolly.pdf
 * (the algorithm is given in the part 5.4.2), which was itself inspired by the method described in
 * the following paper: http://fab.cba.mit.edu/classes/S62.12/docs/Meijster_distance.pdf
 *
 * \param edges The input edge image
 * \param formulation The distance surface formulation that should be used
 *
 * \return The distance surface, computed from the input edge image
 */
Mat distance_surface_cpu(const Mat& edges, string formulation)
{
  // Step 1: computing the values for the lines
  Mat map_x(edges.rows, edges.cols, CV_32S);

  for(int j = 0; j < map_x.rows; ++j) {
    // Setting the value for the first column
    if(edges.at<uint8_t>(j, 0)) {
      map_x.at<int32_t>(j, 0) = 0;
    } else {
      // INT32_MAX is used here as a +∞
      map_x.at<int32_t>(j, 0) = INT32_MAX;
    }

    // Forward pass (going to the right)
    for(int i = 1; i < map_x.cols; ++i) {
      if(edges.at<uint8_t>(j, i)) {
        map_x.at<int32_t>(j, i) = 0;
      } else {
        if(map_x.at<int32_t>(j, i-1) == INT32_MAX) {
          map_x.at<int32_t>(j, i) = INT32_MAX;
        } else {
          map_x.at<int32_t>(j, i) = 1 + map_x.at<int32_t>(j, i-1);
        }
      }
    }

    // Backward pass (going back to the left)
    for(int i = map_x.cols-2; i >= 0; --i) {
      if(map_x.at<int32_t>(j, i+1) < map_x.at<int32_t>(j, i)) {
        map_x.at<int32_t>(j, i) = 1 + map_x.at<int32_t>(j, i+1);
      }
    }
  }

  // Step 2: computing the final map using the one computed in step 1.
  // Note: since the computation is done for each column independantly, we use OpenMP to
  // parallelize the for() loop, to improve the computation time.
  Mat dist_surface(edges.rows, edges.cols, CV_32S);

  #pragma omp parallel for shared(dist_surface, map_x)
  for(int i = 0; i < dist_surface.cols; ++i) {
    // We initialize the various values used for the column
    int q = 0;
    vector<int> s, t;
    s.reserve(dist_surface.rows);
    t.reserve(dist_surface.rows);
    s[0] = 0;
    t[0] = 0;

    // For each row of the i^th column, the best segment of parabola is searched and is added to
    // the stack (or replaces the whole stack if it is better than all the other segments)
    for(int j = 1; j < dist_surface.rows; ++j) {
      while(q >= 0 && parab_ord(map_x, i, s[q], t[q]) > parab_ord(map_x, i, j, t[q])) {
        --q;
      }

      if(q < 0) {
        q = 0;
        s[0] = j;
      } else {
        int32_t parab_inter = parab_inter_abs(map_x, i, s[q], j);
        if(parab_inter != INT32_MAX) {
          int w = 1 + parab_inter;
          if(w >= 0 && w < dist_surface.rows) {
            ++q;
            s[q] = j;
            t[q] = w;
          }
        }
      }
    }

    // Once all the segments for the column were determined, the values are computed and attributed
    // to the cells, and the segments of hyperbolas are removed from the stack once the next best
    // segment is reached
    for(int j = dist_surface.rows-1; j >= 0; --j) {
      dist_surface.at<int32_t>(j, i) = parab_ord(map_x, i, s[q], j);
      if(j == t[q]) {
        --q;
      }
    }
  }

  // Since the value of each cell is the squared distance, we compute the sqrt to obtain the exact
  // euclidian distance
  dist_surface.convertTo(dist_surface, CV_32F);
  cv::sqrt(dist_surface, dist_surface);

  // If a formulation other than the "linear" one is used, a final step has to be computed to apply
  // the correct formulation
  if(formulation != "linear") {
    if(formulation == "linear-bound") {
      // bound_dist_surface = min(dist_surface, saturation_distance)
      dist_surface = min(dist_surface, DISTANCE_SURFACE_SATURATION_DISTANCE);
    } else if(formulation == "logarithmic") {
      // log_dist_surface = log(dist_surface + 1)
      cv::log(dist_surface+1, dist_surface);
    } else if(formulation == "exponential") {
      // exp_dist_surface = 1-exp(-dist_surface/α)
      cv::exp(-dist_surface/ALPHA, dist_surface);
      dist_surface = 1-dist_surface;
    } else {
      ROS_ERROR("Invalid distance surface formulation, exiting!");
      exit(1);
    }
  }

  // And finally, we normalize to output an CV_8U matrix/image
  cv::Mat dist_surface_normalized;
  cv::normalize(dist_surface, dist_surface_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);

  return dist_surface_normalized;
}
