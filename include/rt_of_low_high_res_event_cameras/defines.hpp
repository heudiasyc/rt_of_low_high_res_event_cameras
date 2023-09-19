/**
 * defines.hpp
 * Header file for constant definitions.
 * Feel free to modify this file to adapt it to your needs!
 */

#pragma once

#define METAVISION_SDK_AVAILABLE              false
#define HDF5_SDK_AVAILABLE                    false

#define USE_GPU_VERSION_DEFAULT               true

#define DENOISING_MIN_NEIGHBOURS_DEFAULT      -1
#define FILLING_MIN_NEIGHBOURS_DEFAULT        -1

#define DISTANCE_SURFACE_FORMULATION_DEFAULT  "exponential"
#define DISTANCE_SURFACE_SATURATION_DISTANCE  6
#define ALPHA                                 (DISTANCE_SURFACE_SATURATION_DISTANCE/5.541f)

#define PYRAMIDAL_FLOW_LEVELS_DEFAULT         3
#define MAX_FLOW_DEFAULT                      15.0f
#define GAMMAS_DEFAULT                        {500.0f, 500.0f, 500.0f}
#define SMOOTH_ITERATIONS_DEFAULT             {20, 20, 20}

#define VIZ_DEFAULT                           "colors"
#define CLIP_FLOW_DEFAULT                     -1
#define VIZ_ARROWS_SPREAD                     10
#define VIZ_ARROWS_FACTOR                     20

#define QUEUES_SIZE_DEFAULT                   1
