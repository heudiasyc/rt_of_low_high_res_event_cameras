/**
 * denoising_filling_cpu.hpp
 * Header file for the computation of the denoising and filling step for the edge images, on CPU.
 */

#pragma once

#include "rt_of_low_high_res_event_cameras/includes.hpp"


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
  const Mat& edges, int denoising_min_neighbours, int filling_min_neighbours);
