/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2013 Laurent Kneip, ANU. All rights reserved.      *
 *                                                                            *
 * Redistribution and use in source and binary forms, with or without         *
 * modification, are permitted provided that the following conditions         *
 * are met:                                                                   *
 * * Redistributions of source code must retain the above copyright           *
 *   notice, this list of conditions and the following disclaimer.            *
 * * Redistributions in binary form must reproduce the above copyright        *
 *   notice, this list of conditions and the following disclaimer in the      *
 *   documentation and/or other materials provided with the distribution.     *
 * * Neither the name of ANU nor the names of its contributors may be         *
 *   used to endorse or promote products derived from this software without   *
 *   specific prior written permission.                                       *
 *                                                                            *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
 * ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
 * SUCH DAMAGE.                                                               *
 ******************************************************************************/

/**
 * \file angular.hpp
 * \brief Functions for angular consistency validation for omnidirectional
 *        and panoramic cameras. Replaces traditional cheirality (depth positivity)
 *        constraints with angular error-based validation.
 */

#ifndef OPENGV_ANGULAR_HPP_
#define OPENGV_ANGULAR_HPP_

#include <stdlib.h>
#include <opengv/types.hpp>

/**
 * \brief The namespace of this library.
 */
namespace opengv
{
/**
 * \brief The namespace of the math tools.
 */
namespace math
{

/**
 * \brief Compute angular error between two bearing vectors using cosine similarity.
 *        This is suitable for omnidirectional cameras where bearing vectors can
 *        point in any direction (including backward).
 *
 * \param[in] v1 First bearing vector (should be normalized).
 * \param[in] v2 Second bearing vector (should be normalized).
 * \return Angular error in radians [0, pi].
 */
double angularError(
    const bearingVector_t & v1,
    const bearingVector_t & v2);

/**
 * \brief Check angular consistency between observed and reprojected bearing vectors.
 *        Returns true if the angular error is below the threshold.
 *
 * \param[in] observed Observed bearing vector.
 * \param[in] reprojected Reprojected bearing vector from estimated pose.
 * \param[in] threshold Maximum allowed angular error in radians.
 * \return True if angular error is within threshold, false otherwise.
 */
bool checkAngularConsistency(
    const bearingVector_t & observed,
    const bearingVector_t & reprojected,
    double threshold = 0.1);

/**
 * \brief Compute reprojection error for a point correspondence using angular error.
 *        This replaces traditional reprojection error for omnidirectional cameras.
 *
 * \param[in] bearing Observed bearing vector in camera frame.
 * \param[in] point 3D point in world frame.
 * \param[in] rotation Rotation from camera to world frame.
 * \param[in] translation Camera position in world frame.
 * \return Angular reprojection error in radians.
 */
double angularReprojectionError(
    const bearingVector_t & bearing,
    const point_t & point,
    const rotation_t & rotation,
    const translation_t & translation);

}
}

#endif /* OPENGV_ANGULAR_HPP_ */
