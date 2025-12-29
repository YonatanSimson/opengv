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

#include <opengv/absolute_pose/modules/sqpnp.hpp>
#include <opengv/math/nullspace.hpp>
#include <opengv/math/angular.hpp>
#include <opengv/math/quaternion.hpp>

opengv::absolute_pose::modules::Sqpnp::Sqpnp(void)
{
}

opengv::absolute_pose::modules::Sqpnp::~Sqpnp()
{
}

void
opengv::absolute_pose::modules::Sqpnp::add_correspondence(
    const point_t & worldPoint,
    const bearingVector_t & bearing)
{
  worldPoints.push_back(worldPoint);
  bearings.push_back(bearing);
}

void
opengv::absolute_pose::modules::Sqpnp::reset_correspondences(void)
{
  worldPoints.clear();
  bearings.clear();
}

double
opengv::absolute_pose::modules::Sqpnp::compute_pose(
    rotation_t & R,
    translation_t & t)
{
  if(worldPoints.size() < 4)
    return -1.0;

  // Initialize pose using centroid-based approach
  initialize_pose(R, t);

  // Refine using SQP with null-space constraints
  refine_pose_sqp(R, t);

  // Return final objective value
  return compute_objective(R, t);
}

void
opengv::absolute_pose::modules::Sqpnp::initialize_pose(
    rotation_t & R,
    translation_t & t)
{
  // Use centroid-based initialization with better heuristics
  point_t worldCentroid = point_t::Zero();
  for(size_t i = 0; i < worldPoints.size(); i++)
    worldCentroid += worldPoints[i];
  worldCentroid /= worldPoints.size();

  // Estimate camera-frame centroid from bearings
  bearingVector_t bearingCentroid = bearingVector_t::Zero();
  for(size_t i = 0; i < bearings.size(); i++)
    bearingCentroid += bearings[i];
  bearingCentroid.normalize();

  // Compute initial rotation using Procrustes alignment
  // Build covariance matrix
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for(size_t i = 0; i < worldPoints.size(); i++) {
    point_t pw = worldPoints[i] - worldCentroid;
    bearingVector_t b = bearings[i];
    H += b * pw.transpose();
  }

  // SVD for optimal rotation
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  R = svd.matrixU() * svd.matrixV().transpose();

  // Ensure proper rotation (det = +1)
  if(R.determinant() < 0) {
    Eigen::Matrix3d V = svd.matrixV();
    V.col(2) *= -1;
    R = svd.matrixU() * V.transpose();
  }

  // Estimate translation using least squares
  // For each point: bearing_i * depth_i = R^T * (point_i - t)
  // Solving for t
  Eigen::MatrixXd A(worldPoints.size(), 3);
  Eigen::VectorXd b(worldPoints.size());
  
  for(size_t i = 0; i < worldPoints.size(); i++) {
    A.row(i) = bearings[i].transpose();
    b(i) = bearings[i].dot(R.transpose() * worldPoints[i]);
  }
  
  // Solve least squares: A * (R^T * t) = b
  Eigen::Vector3d Rt_t = A.colPivHouseholderQr().solve(b);
  t = R * Rt_t;
}

void
opengv::absolute_pose::modules::Sqpnp::refine_pose_sqp(
    rotation_t & R,
    translation_t & t,
    int maxIterations)
{
  for(int iter = 0; iter < maxIterations; iter++)
  {
    // Build constraint matrix for null-space computation
    int n = worldPoints.size();
    Eigen::MatrixXd A(n, 6);

    for(int i = 0; i < n; i++)
    {
      point_t p_cam = R.transpose() * (worldPoints[i] - t);
      bearingVector_t b = bearings[i];

      // Construct constraint: bearing x (R^T * (p_w - t)) = 0
      Eigen::Vector3d cross = b.cross(p_cam);

      // Jacobian w.r.t. pose parameters [rotation(3), translation(3)]
      A.block<1, 3>(i, 0) = cross.transpose();
      // For translation part: cross product with each column of R.transpose()
      Eigen::Vector3d dcross_dt;
      dcross_dt(0) = b(1) * R(0, 2) - b(2) * R(0, 1);
      dcross_dt(1) = b(1) * R(1, 2) - b(2) * R(1, 1);
      dcross_dt(2) = b(1) * R(2, 2) - b(2) * R(2, 1);
      A.block<1, 3>(i, 3) = -dcross_dt.transpose();
    }

    // Compute null-space of constraint matrix
    Eigen::MatrixXd nullBasis = opengv::math::computeNullSpace(A);

    if(nullBasis.cols() == 0)
      break; // No null-space, solution converged

    // Orthogonalize null-space basis
    nullBasis = opengv::math::gramSchmidt(nullBasis);

    // Compute gradient of objective
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6);
    for(size_t i = 0; i < worldPoints.size(); i++)
    {
      double error = opengv::math::angularReprojectionError(
          bearings[i], worldPoints[i], R, t);
      
      // Numerical gradient approximation
      double eps = 1e-6;
      for(int j = 0; j < 6; j++)
      {
        Eigen::VectorXd delta = Eigen::VectorXd::Zero(6);
        delta(j) = eps;
        
        rotation_t R_pert = R;
        translation_t t_pert = t;
        
        if(j < 3)
        {
          // Perturb rotation
          Eigen::AngleAxisd aa(eps, Eigen::Vector3d::Unit(j));
          R_pert = aa.toRotationMatrix() * R;
        }
        else
        {
          // Perturb translation
          t_pert(j - 3) += eps;
        }
        
        double error_pert = opengv::math::angularReprojectionError(
            bearings[i], worldPoints[i], R_pert, t_pert);
        
        gradient(j) += (error_pert - error) / eps;
      }
    }

    // Project gradient onto null-space
    Eigen::VectorXd nullGradient = opengv::math::projectToNullSpace(gradient, nullBasis);

    // Update parameters along null-space direction
    double stepSize = 0.01;
    Eigen::VectorXd update = -stepSize * nullGradient;

    // Apply update to rotation (first 3 components)
    if(update.head<3>().norm() > 1e-8)
    {
      Eigen::AngleAxisd aa(update.head<3>().norm(), update.head<3>().normalized());
      R = aa.toRotationMatrix() * R;
    }

    // Apply update to translation (last 3 components)
    t += update.tail<3>();

    // Check convergence
    if(update.norm() < 1e-6)
      break;
  }
}

double
opengv::absolute_pose::modules::Sqpnp::compute_objective(
    const rotation_t & R,
    const translation_t & t)
{
  double totalError = 0.0;
  for(size_t i = 0; i < worldPoints.size(); i++)
  {
    double error = opengv::math::angularReprojectionError(
        bearings[i], worldPoints[i], R, t);
    totalError += error * error;
  }
  return totalError / worldPoints.size();
}
