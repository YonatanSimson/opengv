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

#include <opengv/math/angular.hpp>
#include <cmath>

double
opengv::math::angularError(
    const bearingVector_t & v1,
    const bearingVector_t & v2)
{
  // Compute cosine similarity (dot product for normalized vectors)
  double cosAngle = v1.dot(v2);
  
  // Clamp to valid range to avoid numerical issues with acos
  cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
  
  // Return angle in radians
  return std::acos(cosAngle);
}

bool
opengv::math::checkAngularConsistency(
    const bearingVector_t & observed,
    const bearingVector_t & reprojected,
    double threshold)
{
  double error = angularError(observed, reprojected);
  return error < threshold;
}

double
opengv::math::angularReprojectionError(
    const bearingVector_t & bearing,
    const point_t & point,
    const rotation_t & rotation,
    const translation_t & translation)
{
  // Transform point from world to camera frame
  point_t pointInCamera = rotation.transpose() * (point - translation);
  
  // Compute reprojected bearing vector (normalize)
  bearingVector_t reprojected = pointInCamera.normalized();
  
  // Return angular error
  return angularError(bearing, reprojected);
}
