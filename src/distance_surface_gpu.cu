/**
 * distance_surface_gpu.cpp
 * This code corresponds to the computation of the distance surfaces from the edge images, on
 * CUDA-capable GPU.
 */

#include "rt_of_low_high_res_event_cameras/distance_surface_gpu.hpp"


/**
 * Global variables definition
 */

// A boolean to remember if the GPU memory has already been initialized
bool memory_initialized = false;

// GPU pointers for the various matrices
uint8_t *d_edges, *d_normalized_dist_surface;
int32_t *d_map_x, *d_dist_surface, *d_s, *d_t, *d_dist_surface_cols_max_value;

// Size in bytes of the edge images and the resulting distance surfaces
int edges_bytes, normalized_dist_surface_bytes;

// An int, holding the maximum value of the current distance surface (note that this is a GPU
// variable)
__device__ int32_t d_dist_surface_max_value;


/**
 * \brief GPU kernel function to compute the distance transform along the rows only, which is the
 * first step of the computation of the distance surface.
 *
 * \param d_edges Pointer to the edge image in GPU memory
 * \param d_map_x Pointer to the resulting partial distance surface on the rows, in GPU memory
 * \param nb_rows Number of rows (=height) of the input edge image
 * \param row_size Number of pixels per row (=width) of the input edge image
 */
__global__
void map_x_kernel(const uint8_t* d_edges, int32_t* d_map_x, const int nb_rows, const int row_size)
{
  // We first note the row index and the starting index of the row
  const int row_index = blockIdx.x * blockDim.x + threadIdx.x;
  const int row_first_id = row_index * row_size;

  // We then have to make sure the thread is inside the matrix bounds
  if(row_index < nb_rows) {
    // Setting the value for the first element
    if(d_edges[row_first_id]) {
      d_map_x[row_first_id] = 0;
    } else {
      // INT32_MAX is used here as a +∞
      d_map_x[row_first_id] = INT32_MAX;
    }

    // Forward pass (going to the right)
    for(int i = 1; i < row_size; ++i) {
      if(d_edges[row_first_id+i]) {
        d_map_x[row_first_id+i] = 0;
      } else {
        if(d_map_x[row_first_id+i-1] == INT32_MAX) {
          d_map_x[row_first_id+i] = INT32_MAX;
        } else {
          d_map_x[row_first_id+i] = 1 + d_map_x[row_first_id+i-1];
        }
      }
    }

    // Backward pass (going back to the left)
    for(int i = row_size-2; i >= 0; --i) {
      if(d_map_x[row_first_id+i+1] < d_map_x[row_first_id+i]) {
        d_map_x[row_first_id+i] = 1 + d_map_x[row_first_id+i+1];
      }
    }
  }
}


/**
 * \brief Computes the parabola ordinate (the F^i_y(j) function described in the paper used in the
 * distance transform algorithm, which link is given in the comment of the distance_surface_gpu
 * function)
 *
 * \param y Row of origin of the parabola
 * \param j Row for which we want to compute the ordinate
 * \param map_x_val Ordinate (value) of the parabola at its origin
 *
 * \return The parabola ordinate at the given row coordinates
 */
__device__
int32_t parab_ord(int32_t y, int32_t j, int32_t map_x_val)
{
  if(map_x_val == INT32_MAX) {
    return INT32_MAX;
  }
  return map_x_val*map_x_val + (j-y)*(j-y);
}


/**
 * \brief Computes the abscissa of the intersection of two consecutive parabolas (the Sep^i(u, v)
 * function described in the paper used in the distance transform algorithm, which link is given in
 * the comment of the distance_surface_gpu function)
 *
 * \param u The row (abscissa) of the first parabola
 * \param v The row (abscissa) of the second parabola
 * \param map_x_u_i Ordinate (value) of the first parabola at its origin (u,i)
 * \param map_x_v_i Ordinate (value) of the second parabola at its origin (v,i)
 *
 * \return The abscissa of intersection of the two parabolas
 */
__device__
int32_t parab_inter_abs(int32_t u, int32_t v, int32_t map_x_u_i, int32_t map_x_v_i)
{
  if(map_x_u_i == INT32_MAX || map_x_v_i == INT32_MAX) {
    return INT32_MAX;
  }
  return (v*v - u*u + map_x_v_i*map_x_v_i - map_x_u_i*map_x_u_i) / (2*(v-u));
}


/**
 * \brief GPU kernel function to compute the distance surface, using the partial distance transform
 * on the rows.
 *
 * \param d_map_x Pointer in GPU memory to the input partial distance surface on the rows
 * \param d_dist_surface Pointer in GPU memory to the resulting distance surface
 * \param d_s Pointer in GPU memory to a matrix of the same size as the distance surface, that will
 * be used internaly as the stack of abscissas of parabolas to use when creating the final distance
 * surface image 
 * \param d_t Pointer in GPU memory to a matrix of the same size as the distance surface, that will
 * be used internaly as the stack of abscissas at which the current parabola should not be used
 * anymore, that will be used when creating the final distance surface image
 * \param d_dist_surface_cols_max_value Pointer in GPU memory to an array with the same number of
 * elements as the number of columns (=width) of the distance surface, that will be helpful later
 * on for normalization reasons
 * \param nb_cols Number of columns (=width) of the input edge image
 * \param column_size Number of pixels per column (=height) of the input edge image
 */
__global__
void dist_surf_kernel(
  const int32_t* d_map_x, int32_t* d_dist_surface, int32_t* d_s, int32_t* d_t,
  int32_t* d_dist_surface_cols_max_value, const int nb_cols, const int column_size)
{
  // We first note the column index, its starting index, and the size of each row
  const int column_index = blockIdx.x * blockDim.x + threadIdx.x;
  const int column_first_id = column_index;
  const int row_size = nb_cols;

  // We have to make sure the thread is in the matrix bounds
  if(column_index < nb_cols) {
    // q is used as the s and t stacks index
    int q = 0;
    d_s[column_first_id] = 0;
    d_t[column_first_id] = 0;

    // For each element of the column, the best segment of parabola is searched
    // and is added to the stack (or replaces the whole stack if it is better
    // than all the other segments)
    for(int32_t j = 1; j < column_size; ++j) {
      while(
        q >= 0 && parab_ord(
          d_s[column_first_id+q*row_size], d_t[column_first_id+q*row_size],
          d_map_x[column_first_id + d_s[column_first_id+q*row_size] * row_size])
        > parab_ord(j, d_t[column_first_id+q*row_size], d_map_x[column_first_id+j*row_size]))
      {
        --q;
      }

      if(q < 0) {
        q = 0;
        d_s[column_first_id] = j;
      } else {
        int32_t parab_inter = parab_inter_abs(
          d_s[column_first_id+q*row_size], j,
          d_map_x[column_first_id + d_s[column_first_id + q*row_size] * row_size],
          d_map_x[column_first_id + j*row_size]);
        if(parab_inter != INT32_MAX) {
          int32_t w = 1 + parab_inter;
          if(w >= 0 && w < column_size) {
            ++q;
            d_s[column_first_id+q*row_size] = j;
            d_t[column_first_id+q*row_size] = w;
          }
        }
      }
    }

    // Once all the segments for the column were determined, the values are computed and attributed
    // to the cells, and the segments of hyperbolas are removed from the stack once the next best
    // segment is reached
    for(int32_t j = column_size-1; j >= 0; --j) {
      int32_t cell_value = parab_ord(
        d_s[column_first_id+q*row_size], j,
        d_map_x[column_first_id + d_s[column_first_id+q*row_size] * row_size]);
      d_dist_surface[column_first_id+j*row_size] = cell_value;
      if(j == d_t[column_first_id+q*row_size]) {
        --q;
      }

      // We also try to find here the max value in the column, and we store it for further
      // normalization purposes
      if(j == column_size-1 || cell_value > d_dist_surface_cols_max_value[column_first_id]) {
        d_dist_surface_cols_max_value[column_first_id] = cell_value;
      }
    }
  }
}


/**
 * \brief Finds the maximum value of the distance surface matrix, based on the maximum value of
 * each of its columns (computed in the dist_surf_kernel)
 *
 * \param d_dist_surface_cols_max_value Pointer in GPU memory to an array containing for each cell
 * the maximum value of the corresponding column of the distance surface computed using the
 * dist_surf_kernel function
 * \param nb_cols Number of columns of the distance surface
 */
__global__
void find_dist_surface_max_value(const int32_t* d_dist_surface_cols_max_value, const int nb_cols)
{
  int32_t max = 0;
  for(int i = 0; i < nb_cols; i++) {
    if(d_dist_surface_cols_max_value[i] > max) {
      max = d_dist_surface_cols_max_value[i];
    }
  }
  d_dist_surface_max_value = max;
}


/**
 * \brief GPU kernel function to normalize the distance surface to an uint8_t image (values between
 * 0 and 255), and to apply the correct distance surface formulation (linear, exponential, ...).
 * This is the third (and final) step of the computation of the distance surface.
 *
 * \param d_dist_surface Pointer in GPU memory to the input distance surface
 * \param d_normalized_dist_surface Pointer in GPU memory to the output normalized distance surface
 * \param nb_elems Number of pixels in the distance surface image
 * \param formulation The distance surface formulation that should be used, represented as an
 * integer
 */
__global__
void ds_norm_kernel(
  const int32_t* d_dist_surface, uint8_t* d_normalized_dist_surface, const int nb_elems,
  uint8_t formulation)
{
  // We first note the pixel index, and the stride that should be used
  const int index = blockIdx.x * blockDim.x + threadIdx.x;
  const int stride = blockDim.x * gridDim.x;

  // We then normalize the distance surface, using the correct formulation, and using the
  // previously computed maximum value of the distance surface
  if(formulation == 0) { // Linear
    for(int i = index; i < nb_elems; i += stride) {
      // The sqrt here is because the dist_surface originally contains the squared distance, but we
      // want here to have the Euclidean distance.
      // Other note: the ceil() is important here, because we don't want values close to 0 to be
      // rounded to 0. By using ceil(), we make sure than any value slightly superior to 0 will be
      // set to 1 (and the call to min() prevents any overflow, which can sometimes happen
      // strangely??)
      d_normalized_dist_surface[i] = min(
        ceil(sqrtf((float)d_dist_surface[i] / d_dist_surface_max_value) * 255), 255.);
    }
  } else if(formulation == 1) { // Linear bound
    float max = min(sqrtf(d_dist_surface_max_value), (float)DISTANCE_SURFACE_SATURATION_DISTANCE);
    for(int i = index; i < nb_elems; i += stride) {
      d_normalized_dist_surface[i] = min(
        ceil(
          min(sqrtf(d_dist_surface[i]), (float)DISTANCE_SURFACE_SATURATION_DISTANCE) / max * 255),
        255.);
    }
  } else if(formulation == 2) { // Logarithmic
    float max = logf(sqrtf(d_dist_surface_max_value)+1);
    for(int i = index; i < nb_elems; i += stride) {
      d_normalized_dist_surface[i] = min(ceil(logf(sqrtf(d_dist_surface[i])+1) / max * 255), 255.);
    }
  } else if(formulation == 3) { // Exponential
    for(int i = index; i < nb_elems; i += stride) {
      d_normalized_dist_surface[i] = min(
        (1-expf(-sqrtf(d_dist_surface[i])/ALPHA)) * 255, 255.);
    }
  } else {
    printf("Wrong distance surface formulation index!\n");
  }
}


/**
 * \brief Allocates the adequate memory on the GPU.
 *
 * \param edges An edge image, used to compute the number of bytes that should be reserved on the
 * GPU
 */
void init_cuda_memory(const Mat& edges)
{
  // The matrices that will be used are:
  // - the edge image, of type uint8_t
  // - the partial distance surface on the rows only (map_x), of type int32_t
  // - the stacks for keeping the correct parabolas, s and t, of type int32_t
  // - an array keeping the max value per column, of type int32_t
  // - the output normalized distance surface, of type uint8_t

  // Calculating the total number of bytes per matrix
  edges_bytes = edges.step * edges.rows;
  const int map_x_bytes = edges_bytes * 4;
  const int dist_surface_bytes = edges_bytes * 4;
  const int s_t_bytes = edges_bytes * 4;
  const int dist_surface_cols_max_value_bytes = edges.cols * 4;
  normalized_dist_surface_bytes = edges_bytes;

  // Allocating memory on the GPU
  cudaMalloc<uint8_t>(&d_edges, edges_bytes);
  cudaMalloc<int32_t>(&d_map_x, map_x_bytes);
  cudaMalloc<int32_t>(&d_dist_surface, dist_surface_bytes);
  cudaMalloc<int32_t>(&d_s, s_t_bytes);
  cudaMalloc<int32_t>(&d_t, s_t_bytes);
  cudaMalloc<int32_t>(&d_dist_surface_cols_max_value,
                      dist_surface_cols_max_value_bytes);
  cudaMalloc<uint8_t>(&d_normalized_dist_surface,
                      normalized_dist_surface_bytes);
}


/**
 * \brief Frees the memory allocated on GPU.
 * It must be called manually at the end of the execution!
 */
void free_gpu_memory()
{
  // Freeing the memory allocated to all matrices if needed
  if(memory_initialized) {
    cudaFree(d_edges);
    cudaFree(d_map_x);
    cudaFree(d_dist_surface);
    cudaFree(d_s);
    cudaFree(d_t);
    cudaFree(d_dist_surface_cols_max_value);
    cudaFree(d_normalized_dist_surface);
  }
}


/**
 * \brief Computes the distance surface from an edge image, on GPU, using the exact method
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
Mat distance_surface_gpu(const Mat& edges, string formulation)
{
  // Before beginning, we must check that the given formulation for the distance surface is
  // correct; but also, since CUDA kernel function do not handle C++ strings, we convert it to an
  // integer representation
  uint8_t formulation_nbr;
  if(formulation == "linear") {
    formulation_nbr = 0;
  } else if(formulation == "linear-bound") {
    formulation_nbr = 1;
  } else if(formulation == "logarithmic") {
    formulation_nbr = 2;
  } else if(formulation == "exponential") {
    formulation_nbr = 3;
  } else {
    ROS_ERROR("Invalid distance surface formulation, exiting!");
    exit(1);
  }

  // Before beginning, we also have to make sure memory is already initialized
  if(!memory_initialized) {
    init_cuda_memory(edges);
    memory_initialized = true;
  }

  // Step 1: computing the distance values for the rows of the edge image
  // The computation for each row is done in // on the GPU

  // We first copy data from the edges Mat to device memory
  cudaMemcpy(d_edges, edges.ptr(), edges_bytes, cudaMemcpyHostToDevice);

  // We then launch the map_x computation kernel, as a set of blocks, each block containing
  // 64 threads
  const int threads_per_block_map_x = 64;
  const int nb_blocks_map_x = (edges.rows + threads_per_block_map_x - 1) / threads_per_block_map_x;
  map_x_kernel<<<nb_blocks_map_x, threads_per_block_map_x>>>(
    d_edges, d_map_x, edges.rows, edges.cols);

  // And we wait for the computation to finish
  cudaDeviceSynchronize();

  // Step 2: computing the final distance surface using the partial one computed in step 1

  // We launch the distance surface computation kernel, as a set of blocks, each block containing
  // 64 threads
  const int threads_per_block_ds = 64;
  const int nb_blocks_ds = (edges.cols + threads_per_block_ds - 1) / threads_per_block_ds;
  dist_surf_kernel<<<nb_blocks_ds, threads_per_block_ds>>>(
    d_map_x, d_dist_surface, d_s, d_t, d_dist_surface_cols_max_value, edges.cols, edges.rows);

  // And we wait for the computation to finish
  cudaDeviceSynchronize();

  // Step 3: normalizing the distance surface. This step has two purposes:
  // 1) creating an uint8_t image, that is, an image which values are integers between 0 and 255
  // 2) applying the specific required formulation for the distance surface (linear, linear-bound,
  //    logarithmic, or exponential)

  // For every formulation other than the inverse exponential one, we first have to find the
  // maximum value of the distance surface. For that purpose, we use the previously maximum values
  // found for each column.
  // Note also that a GPU kernel is used here, but without any parallelization. We could use a CPU
  // function, but since these maximum values are stored in the GPU memory, we want to avoid
  // useless copies, hence the GPU kernel.
  if(formulation_nbr != 3) {
    find_dist_surface_max_value<<<1, 1>>>(d_dist_surface_cols_max_value, edges.cols);

    // And we wait for the computation to finish
    cudaDeviceSynchronize();
  }

  // We then launch the distance surface normalization kernel, as a set of blocks, each block
  // containing 64 threads
  const int threads_per_block_norm = 64;
  const int nb_blocks_norm = (edges.cols*edges.rows + threads_per_block_norm - 1)
    / threads_per_block_norm;
  ds_norm_kernel<<<nb_blocks_norm, threads_per_block_norm>>>(
    d_dist_surface, d_normalized_dist_surface, edges.cols*edges.rows, formulation_nbr);

  // And we wait once again for the computation to finish
  cudaDeviceSynchronize();

  // Finally, we copy the normalized distance surface from GPU to host memory
  Mat dist_surface(edges.rows, edges.cols, CV_8U);
  cudaMemcpy(
    dist_surface.ptr(), d_normalized_dist_surface, normalized_dist_surface_bytes,
    cudaMemcpyDeviceToHost);

  // And we return the matrix
  return dist_surface;
}
