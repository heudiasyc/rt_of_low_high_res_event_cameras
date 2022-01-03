/**
 * denoising_filling_cpu.hpp
 * Header file for the computation of the denoising and filling step for the edge images, on CPU.
 */

#pragma once

#include "rt_of_low_high_res_event_cameras/includes.hpp"


/**
 * \brief Computes the denoised & filled version of the input edge image, using the GPU.
 * 
 * \param edges The noisy edge image
 * \param denoising_min_neighbours Threshold for the denoising step (N_d)
 * \param filling_min_neighbours Threshold for the filling step (N_f)
 * 
 * \return A denoised and filled copy of the input edge image
 */
Mat denoising_filling_gpu(
  const Mat& edges, int filtering_min_neighbours, int smoothing_min_neighbours);


/**
 * \brief Frees the memory allocated on GPU.
 * It must be called manually at the end of the execution!
 */
void free_gpu_memory();
