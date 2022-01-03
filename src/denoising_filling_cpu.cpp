/**
 * denoising_filling_cpu.cpp
 * This code corresponds to the computation of the denoising and filling step for the edge images,
 * on CPU.
 */

#include "rt_of_low_high_res_event_cameras/denoising_filling_cpu.hpp"


/**
 * \brief Computes the denoised & filled version of the input edge image, using the CPU.
 * 
 * \param edges The noisy edge image
 * \param denoising_min_neighbours Threshold for the denoising step (N_d)
 * \param filling_min_neighbours Threshold for the filling step (N_f)
 * 
 * \return A denoised and filled copy of the input edge image
 */
Mat denoising_filling_cpu(
  const Mat& edges, int denoising_min_neighbours, int filling_min_neighbours)
{
  // Initializing the denoised edge image as a copy of the original edge image
  Mat denoised_edges;
  edges.copyTo(denoised_edges);

  // Applying the denoising (if needed)
  if(denoising_min_neighbours > 0 && denoising_min_neighbours <= 4) {
    // For each pixel (not too close to the border)...
    for(int y = 1; y < edges.rows-1; ++y) {
      for(int x = 1; x < edges.cols-1; ++x) {
        // ... if there is an edge...
        if(edges.at<uint8_t>(y, x)) {
          // ... we count the number of *direct* neighbours.
          int nb_neighbours = 0;
          for(int j = -1; j <= 1; ++j) {
            for(int i = -1; i <= 1; ++i) {
              if((i == 0 || j == 0) && i != j) {
                if(edges.at<uint8_t>(y+j, x+i)) {
                  ++nb_neighbours;
                }
              }
            }
          }
          // If not enough were found: the edge pixel is discarded
          if(nb_neighbours < denoising_min_neighbours) {
            denoised_edges.at<uint8_t>(y, x) = 0;
          }
        }
      }
    }
  }

  // Initializing the filled edge image as a copy of the denoised edge image
  Mat filled_edges;
  denoised_edges.copyTo(filled_edges);

  // Applying the filling (if needed)
  if(filling_min_neighbours > 0 && filling_min_neighbours <= 4) {
    // For each pixel (not too close to the border)...
    for(int y = 1; y < denoised_edges.rows-1; ++y) {
      for(int x = 1; x < denoised_edges.cols-1; ++x) {
        // ... if there is no edge...
        if(!denoised_edges.at<uint8_t>(y, x)) {
          // ... we count the number of *direct* neighbours.
          int nb_neighbours = 0;
          for(int j = -1; j <= 1; ++j) {
            for(int i = -1; i <= 1; ++i) {
              if((i == 0 || j == 0) && i != j) {
                if(denoised_edges.at<uint8_t>(y+j, x+i)) {
                  ++nb_neighbours;
                }
              }
            }
          }
          // If enough were found: a new edge pixel is added
          if(nb_neighbours >= filling_min_neighbours) {
            filled_edges.at<uint8_t>(y, x) = 255;
          }
        }
      }
    }
  }

  // Finally, we return the denoised and filled edge image
  return filled_edges;
}
