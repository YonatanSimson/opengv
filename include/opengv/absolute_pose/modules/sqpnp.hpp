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
 * \file sqpnp.hpp
 * \brief SQPnP (Sequential Quadratic Programming PnP) solver for absolute pose
 *        estimation with null-space refinement. Suitable for omnidirectional
 *        and panoramic cameras.
 */

#ifndef OPENGV_ABSOLUTE_POSE_MODULES_SQPNP_HPP_
#define OPENGV_ABSOLUTE_POSE_MODULES_SQPNP_HPP_

#include <stdlib.h>
#include <Eigen/Eigen>
#include <opengv/types.hpp>

namespace opengv
{
namespace absolute_pose
{
namespace modules
{

/**
 * \brief SQPnP solver implementing null-space-based pose refinement
 *        with sequential quadratic programming.
 */
class Sqpnp
{
public:
  Sqpnp(void);
  ~Sqpnp();

  /**
   * \brief Add a correspondence between a 3D point and bearing vector.
   *
   * \param[in] worldPoint 3D point in world frame.
   * \param[in] bearing Normalized bearing vector in camera frame.
   */
  void add_correspondence(
      const point_t & worldPoint,
      const bearingVector_t & bearing);

  /**
   * \brief Compute the pose using SQPnP with null-space refinement.
   *
   * \param[out] R Output rotation matrix (camera to world).
   * \param[out] t Output translation vector (camera position in world).
   * \return Residual error after optimization.
   */
  double compute_pose(rotation_t & R, translation_t & t);

  /**
   * \brief Reset correspondences for reuse.
   */
  void reset_correspondences(void);

private:
  /**
   * \brief Initialize the pose estimate using a subset of points.
   */
  void initialize_pose(rotation_t & R, translation_t & t);

  /**
   * \brief Refine pose using null-space-based SQP optimization.
   */
  void refine_pose_sqp(rotation_t & R, translation_t & t, int maxIterations = 10);

  /**
   * \brief Compute objective function value (angular reprojection error).
   */
  double compute_objective(const rotation_t & R, const translation_t & t);

  std::vector<point_t> worldPoints;
  std::vector<bearingVector_t> bearings;
};

}
}
}

#endif /* OPENGV_ABSOLUTE_POSE_MODULES_SQPNP_HPP_ */
