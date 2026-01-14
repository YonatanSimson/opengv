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

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <limits.h>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <Eigen/Eigen>
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac/Lmeds.hpp>
#include <opengv/sac_problems/absolute_pose/AbsolutePoseSacProblem.hpp>
#include <sstream>
#include <fstream>

#include "random_generators.hpp"
#include "experiment_helpers.hpp"
#include "time_measurement.hpp"


using namespace std;
using namespace Eigen;
using namespace opengv;

int main( int argc, char** argv )
{
  //initialize random seed
  initializeRandomSeed();
  
  //set experiment parameters
  double noise = 0.5;  // Add noise to see refinement differences
  double outlierFraction = 0.1;
  size_t numberPoints = 100;

  //create a random viewpoint pose
  translation_t position = generateRandomTranslation(2.0);
  rotation_t rotation = generateRandomRotation(0.5);
  
  //create a fake central camera
  translations_t camOffsets;
  rotations_t camRotations;
  generateCentralCameraSystem( camOffsets, camRotations );
  
  //derive correspondences based on random point-cloud
  bearingVectors_t bearingVectors;
  points_t points;
  std::vector<int> camCorrespondences; //unused in the central case!
  Eigen::MatrixXd gt(3,numberPoints);
  generateRandom2D3DCorrespondences(
      position, rotation, camOffsets, camRotations, numberPoints, noise, outlierFraction,
      bearingVectors, points, camCorrespondences, gt );

  //print the experiment characteristics
  printExperimentCharacteristics(
      position, rotation, noise, outlierFraction );

  //create a central absolute adapter
  absolute_pose::CentralAbsoluteAdapter adapter(
      bearingVectors,
      points,
      rotation);

  //Create an AbsolutePoseSac problem and Ransac
  //The method can be set to KNEIP, GAO or EPNP
  sac::Ransac<sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
  std::shared_ptr<
      sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
      new sac_problems::absolute_pose::AbsolutePoseSacProblem(
      adapter,
      sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP));
  ransac.sac_model_ = absposeproblem_ptr;
  ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  ransac.max_iterations_ = 50;

  //Run the experiment
  struct timeval tic;
  struct timeval toc;
  gettimeofday( &tic, 0 );
  ransac.computeModel();
  gettimeofday( &toc, 0 );
  double ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));

  //print the results
  std::cout << "the ransac results is: " << std::endl;
  std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
  std::cout << ransac_time << " seconds" << std::endl << std::endl;
  std::cout << "the number of inliers is: " << ransac.inliers_.size();
  std::cout << std::endl << std::endl;
  std::cout << "the found inliers are: " << std::endl;
  for(size_t i = 0; i < ransac.inliers_.size(); i++)
    std::cout << ransac.inliers_[i] << " ";
  std::cout << std::endl << std::endl;

  // Create LMedS
  sac::Lmeds<sac_problems::absolute_pose::AbsolutePoseSacProblem> lmeds;
  lmeds.sac_model_ = absposeproblem_ptr;
  lmeds.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/800.0));
  lmeds.max_iterations_ = 50;

  //Run the LMedS experiment
  gettimeofday( &tic, 0 );
  lmeds.computeModel();
  gettimeofday( &toc, 0 );
  double lmeds_time = TIMETODOUBLE(timeval_minus(toc,tic));

  //print the results
  std::cout << "the lmeds results is: " << std::endl;
  std::cout << lmeds.model_coefficients_ << std::endl << std::endl;
  std::cout << "Lmeds needed " << lmeds.iterations_ << " iterations and ";
  std::cout << lmeds_time << " seconds" << std::endl << std::endl;
  std::cout << "the number of inliers is: " << lmeds.inliers_.size();
  std::cout << std::endl << std::endl;
  std::cout << "the found inliers are: " << std::endl;
  for(size_t i = 0; i < lmeds.inliers_.size(); i++)
    std::cout << lmeds.inliers_[i] << " ";
  std::cout << std::endl << std::endl;

  //==========================================================================
  // COMPARISON: LM vs SQPNP refinement after RANSAC
  //==========================================================================
  std::cout << "==================================================" << std::endl;
  std::cout << "Comparing Refinement Methods: LM vs SQPNP" << std::endl;
  std::cout << "(Both use KNEIP P3P RANSAC for outlier rejection)" << std::endl;
  std::cout << "==================================================" << std::endl;
  
  // Ground truth
  transformation_t gt_transformation;
  gt_transformation.col(3) = position;
  gt_transformation.block<3,3>(0,0) = rotation;
  
  std::cout << "Ground truth:" << std::endl;
  std::cout << gt_transformation << std::endl << std::endl;
  
  std::cout << "RANSAC raw result (before refinement):" << std::endl;
  std::cout << ransac.model_coefficients_ << std::endl << std::endl;
  
  // Create adapter with inlier correspondences for refinement
  bearingVectors_t inlier_bearings;
  points_t inlier_points;
  for(size_t i = 0; i < ransac.inliers_.size(); i++)
  {
    int idx = ransac.inliers_[i];
    inlier_bearings.push_back(bearingVectors[idx]);
    inlier_points.push_back(points[idx]);
  }
  
  absolute_pose::CentralAbsoluteAdapter inlier_adapter(
      inlier_bearings,
      inlier_points);
  
  //--------------------------------------------------------------------------
  // Method 1: LM (Levenberg-Marquardt) refinement - optimize_nonlinear
  //--------------------------------------------------------------------------
  inlier_adapter.sett(ransac.model_coefficients_.col(3));
  inlier_adapter.setR(ransac.model_coefficients_.block<3,3>(0,0));
  
  gettimeofday( &tic, 0 );
  transformation_t lm_refined = absolute_pose::optimize_nonlinear(inlier_adapter);
  gettimeofday( &toc, 0 );
  double lm_refine_time = TIMETODOUBLE(timeval_minus(toc,tic));
  
  //--------------------------------------------------------------------------
  // Method 2: SQPNP refinement
  //--------------------------------------------------------------------------
  gettimeofday( &tic, 0 );
  transformation_t sqpnp_refined = absolute_pose::sqpnp(inlier_adapter);
  gettimeofday( &toc, 0 );
  double sqpnp_refine_time = TIMETODOUBLE(timeval_minus(toc,tic));
  
  //--------------------------------------------------------------------------
  // Compute errors for all methods
  //--------------------------------------------------------------------------
  
  // Position errors
  double ransac_pos_error = (ransac.model_coefficients_.col(3) - position).norm();
  double lm_pos_error = (lm_refined.col(3) - position).norm();
  double sqpnp_pos_error = (sqpnp_refined.col(3) - position).norm();
  
  // Rotation errors (Rodrigues vector norm)
  rotation_t ransac_R = ransac.model_coefficients_.block<3,3>(0,0);
  rotation_t ransac_R_rel = ransac_R.transpose() * rotation;
  Eigen::AngleAxisd ransac_aa(ransac_R_rel);
  double ransac_rot_error = ransac_aa.angle();
  
  rotation_t lm_R = lm_refined.block<3,3>(0,0);
  rotation_t lm_R_rel = lm_R.transpose() * rotation;
  Eigen::AngleAxisd lm_aa(lm_R_rel);
  double lm_rot_error = lm_aa.angle();
  
  rotation_t sqpnp_R = sqpnp_refined.block<3,3>(0,0);
  rotation_t sqpnp_R_rel = sqpnp_R.transpose() * rotation;
  Eigen::AngleAxisd sqpnp_aa(sqpnp_R_rel);
  double sqpnp_rot_error = sqpnp_aa.angle();
  
  //--------------------------------------------------------------------------
  // Print comparison results
  //--------------------------------------------------------------------------
  std::cout << "==================================================" << std::endl;
  std::cout << "REFINEMENT COMPARISON (using " << ransac.inliers_.size() << " inliers)" << std::endl;
  std::cout << "==================================================" << std::endl;
  std::cout << std::fixed << std::setprecision(8);
  
  std::cout << std::endl << "LM refined result:" << std::endl;
  std::cout << lm_refined << std::endl;
  
  std::cout << std::endl << "SQPNP refined result:" << std::endl;
  std::cout << sqpnp_refined << std::endl;
  
  // Convert angular errors to degrees
  const double rad2deg = 180.0 / M_PI;
  double ransac_rot_error_deg = ransac_rot_error * rad2deg;
  double lm_rot_error_deg = lm_rot_error * rad2deg;
  double sqpnp_rot_error_deg = sqpnp_rot_error * rad2deg;
  
  std::cout << std::endl << "==================================================" << std::endl;
  std::cout << "ERROR METRICS" << std::endl;
  std::cout << "==================================================" << std::endl;
  std::cout << std::setw(18) << "Method" 
            << std::setw(16) << "Pos Err (m)" 
            << std::setw(16) << "Ang Err (rad)"
            << std::setw(16) << "Ang Err (deg)"
            << std::setw(12) << "Time (ms)" << std::endl;
  std::cout << std::setw(18) << "--------------" 
            << std::setw(16) << "-----------" 
            << std::setw(16) << "------------"
            << std::setw(16) << "------------"
            << std::setw(12) << "--------" << std::endl;
  std::cout << std::setw(18) << "RANSAC (raw)" 
            << std::setw(16) << ransac_pos_error 
            << std::setw(16) << ransac_rot_error 
            << std::setw(16) << ransac_rot_error_deg
            << std::setw(12) << (ransac_time * 1000.0) << std::endl;
  std::cout << std::setw(18) << "LM refined" 
            << std::setw(16) << lm_pos_error 
            << std::setw(16) << lm_rot_error 
            << std::setw(16) << lm_rot_error_deg
            << std::setw(12) << (lm_refine_time * 1000.0) << std::endl;
  std::cout << std::setw(18) << "SQPNP refined" 
            << std::setw(16) << sqpnp_pos_error 
            << std::setw(16) << sqpnp_rot_error 
            << std::setw(16) << sqpnp_rot_error_deg
            << std::setw(12) << (sqpnp_refine_time * 1000.0) << std::endl;
  
  std::cout << std::endl << "==================================================" << std::endl;
  std::cout << "CONCLUSION" << std::endl;
  std::cout << "==================================================" << std::endl;
  if(lm_pos_error < sqpnp_pos_error && lm_rot_error < sqpnp_rot_error)
    std::cout << "LM refinement performed better on this run." << std::endl;
  else if(sqpnp_pos_error < lm_pos_error && sqpnp_rot_error < lm_rot_error)
    std::cout << "SQPNP refinement performed better on this run." << std::endl;
  else
    std::cout << "Mixed results: one method better for position, other for rotation." << std::endl;
  
  std::cout << std::endl << "Note: For omnidirectional/panorama cameras with 360-degree bearing" << std::endl;
  std::cout << "vectors, SQPNP refinement is preferred as it properly handles" << std::endl;
  std::cout << "full 3D bearing vectors (LM uses perspective reprojection error)." << std::endl;
  std::cout << std::endl;
}
