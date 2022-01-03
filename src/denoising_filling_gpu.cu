/**
 * denoising_filling_gpu.cu
 * This code corresponds to the computation of the denoising and filling step for the edge images,
 * on CUDA-capable GPU.
 */

#include "rt_of_low_high_res_event_cameras/denoising_filling_gpu.hpp"


/**
 * Global variables definition
 */

// A boolean to remember if the GPU memory has already been initialized
bool memory_initialized = false;

// GPU pointers for the edge image, the denoised edge image, and the filled edge image respectively
uint8_t *d_edges, *d_denoised_edges, *d_filled_edges;

// Size in bytes of the edge images 
int edges_bytes;


/**
 * \brief GPU kernel function, which computes the denoised edge image.
 * 
 * \param d_edges Pointer to the edge image, in GPU memory
 * \param d_denoised_edges Pointer to the denoised edge image, in GPU memory
 * \param nb_rows Number of rows of the edge image
 * \param nb_cols Number of columns of the edge image
 * \param denoising_min_neighbours Denoising threshold (N_d)
 */
__global__
void denoising_kernel(
  const uint8_t* d_edges, uint8_t* d_denoised_edges, const int nb_rows, const int nb_cols,
  const int denoising_min_neighbours)
{
  // We first compute the thread index and the stride to go through the whole edge image
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;

  // We then go through all the pixels asigned to this thread
  for(int i = index; i < nb_rows*nb_cols; i += stride) {
    // We apply denoising only if necessary
    if(denoising_min_neighbours > 0 && denoising_min_neighbours <= 4) {
      uint8_t val = 0;

      // If this pixel has an event (i.e. if it is an edge pixel)...
      if(d_edges[i]) {
        // ... we compute the row and column indices...
        const int row = i / nb_cols;
        const int col = i % nb_cols;

        // ... we make sure we are not too close to the border...
        if(row > 0 && row < nb_rows-1 && col > 0 && col < nb_cols-1) {
          // ... and we compute the number of direct neighbours that are also edge pixels.
          uint8_t nb_neighbours = 0;
          for(int y = -1; y <= 1; ++y) {
            for(int x = -1; x <= 1; ++x) {
              if((x == 0 || y == 0) && x != y) {
                if(d_edges[i+y*nb_cols+x]) {
                  ++nb_neighbours;
                }
              }
            }
          }

          // If there are enough correct neighbours, the edge pixel is kept
          if(nb_neighbours >= denoising_min_neighbours) {
            val = 255;
          }
        }
      }

      // We asign the final value to the pixel
      d_denoised_edges[i] = val;
    } else {
      // If denoising wasn't asked, we simply copy the data
      d_denoised_edges[i] = d_edges[i];
    }
  }
}


/**
 * \brief GPU kernel function, which computes the filled edge image.
 * 
 * \param d_denoised_edges Pointer to the denoised edge image, in GPU memory
 * \param d_filled_edges Pointer to the filled edge image, in GPU memory
 * \param nb_rows Number of rows of the edge image
 * \param nb_cols Number of columns of the edge image
 * \param filling_min_neighbours Filling threshold (N_f)
 */
__global__
void filling_kernel(
  const uint8_t* d_denoised_edges, uint8_t* d_filled_edges, const int nb_rows, const int nb_cols,
  const int filling_min_neighbours)
{
  // We first compute the thread index and the stride to go through the whole denoised edge image
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;

  // We go through all the pixels asigned to this thread
  for(int i = index; i < nb_rows*nb_cols; i += stride) {
    // We apply filling only if necessary
    if(filling_min_neighbours > 0 && filling_min_neighbours <= 4) {
      uint8_t val = 0;

      // If this pixel has no event (i.e. if it is not an edge pixel)...
      if(!d_denoised_edges[i]) {
        // ... we compute the row and column indices...
        const int row = i / nb_cols;
        const int col = i % nb_cols;

        // ... we make sure we are not too close to the border...
        if(row > 0 && row < nb_rows-1 && col > 0 && col < nb_cols-1) {
          // ... and we compute the number of direct neighbours that are edge pixels.
          uint8_t nb_neighbours = 0;
          for(int y = -1; y <= 1; ++y) {
            for(int x = -1; x <= 1; ++x) {
              if((x == 0 || y == 0) && x != y) {
                if(d_denoised_edges[i+y*nb_cols+x]) {
                  ++nb_neighbours;
                }
              }
            }
          }

          // If there are enough correct neighbours, the pixel becomes a new edge pixel
          if(nb_neighbours >= filling_min_neighbours) {
            val = 255;
          }
        }
      } else {
        // If the pixel was already an edge pixel, then it should remain one
        val = 255;
      }

      // We asign the final value to the pixel
      d_filled_edges[i] = val;
    } else {
      // If filling wasn't asked, we simply copy the data
      d_filled_edges[i] = d_denoised_edges[i];
    }
  }
}


/**
 * \brief Allocates the adequate memory on the GPU.
 * 
 * \param edges An edge image, used to compute the number of bytes that should be reserved on the
 * GPU
 */
void init_gpu_memory(const Mat& edges)
{
  // Calculating the total size in bytes of the edges matrix
  edges_bytes = edges.step * edges.rows;

  // Allocating memory on the GPU, for the three matrices (edges, denoised_edges, filled_edges)
  cudaMalloc<uint8_t>(&d_edges, edges_bytes);
  cudaMalloc<uint8_t>(&d_denoised_edges, edges_bytes);
  cudaMalloc<uint8_t>(&d_filled_edges, edges_bytes);
}


/**
 * \brief Frees the memory allocated on GPU.
 * It must be called manually at the end of the execution!
 */
void free_gpu_memory()
{
  // Freeing the memory allocated to all the matrices if needed
  if(memory_initialized) {
    cudaFree(d_edges);
    cudaFree(d_denoised_edges);
    cudaFree(d_filled_edges);
  }
}


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
  const Mat& edges, int denoising_min_neighbours, int filling_min_neighbours)
{
  // Before doing anything, we have to make sure memory is already initialized
  if(!memory_initialized) {
    init_gpu_memory(edges);
    memory_initialized = true;
  }

  // Step 1: Denoising
  // The computation for each pixel is done in // on the GPU

  // We first copy data from the edges Mat to device memory
  cudaMemcpy(d_edges, edges.ptr(), edges_bytes, cudaMemcpyHostToDevice);

  // We then launch the denoising kernel, as a set of blocks, each block containing 64 threads
  const int threads_per_block_d = 64;
  const int nb_blocks_d = (edges.rows*edges.cols + threads_per_block_d - 1) / threads_per_block_d;
  denoising_kernel<<<nb_blocks_d, threads_per_block_d>>>(
    d_edges, d_denoised_edges, edges.rows, edges.cols, denoising_min_neighbours);

  // And we wait for the computation to finish
  cudaDeviceSynchronize();

  // Step 2: filling
  // The computation for each pixel is also done in // on the GPU

  // No need to copy the denoised edge image back in host memory, as it can stay on the GPU to be
  // directly used for the filling step. So, we launch the filling kernel, as a set of blocks, each
  // block containing 64 threads
  const int threads_per_block_f = 64;
  const int nb_blocks_f = (edges.rows*edges.cols + threads_per_block_f - 1) / threads_per_block_f;
  filling_kernel<<<nb_blocks_f, threads_per_block_f>>>(
    d_denoised_edges, d_filled_edges, edges.rows, edges.cols, filling_min_neighbours);

  // And we wait for the computation to finish once again
  cudaDeviceSynchronize();

  // We finally copy the denoised & filled edge image from GPU to host memory
  Mat filled_edges(edges.rows, edges.cols, CV_8U);
  cudaMemcpy(filled_edges.ptr(), d_filled_edges, edges_bytes, cudaMemcpyDeviceToHost);

  // And we return the matrix
  return filled_edges;
}
