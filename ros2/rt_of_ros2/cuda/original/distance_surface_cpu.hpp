/**
 * distance_surface_cpu.hpp
 * Header file for the computation of the distance surfaces from the edge images, on CPU.
 */

#pragma once

#include "rt_of_low_high_res_event_cameras/defines.hpp"
#include "rt_of_low_high_res_event_cameras/includes.hpp"


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
Mat distance_surface_cpu(const Mat& edges, string formulation);
