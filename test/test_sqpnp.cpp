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
#include <opengv/absolute_pose/methods.hpp>
#include <opengv/absolute_pose/CentralAbsoluteAdapter.hpp>
#include <opengv/math/angular.hpp>

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
  double noise = 0.0;
  double outlierFraction = 0.0;
  size_t numberPoints = 20;

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
  std::vector<int> camCorrespondences;
  Eigen::MatrixXd gt(3,numberPoints);
  generateRandom2D3DCorrespondences(
      position, rotation, camOffsets, camRotations, numberPoints, noise, outlierFraction,
      bearingVectors, points, camCorrespondences, gt );

  //print the experiment characteristics
  std::cout << "Testing SQPnP with omnidirectional support" << std::endl;
  std::cout << "===========================================" << std::endl;
  printExperimentCharacteristics(
      position, rotation, noise, outlierFraction );

  //create a central absolute adapter
  absolute_pose::CentralAbsoluteAdapter adapter(
      bearingVectors,
      points,
      rotation );

  //timer
  struct timeval tic;
  struct timeval toc;
  size_t iterations = 10;

  //run SQPnP test
  std::cout << "running SQPnP (all correspondences)" << std::endl;
  transformation_t sqpnp_transformation;
  gettimeofday( &tic, 0 );
  for(size_t i = 0; i < iterations; i++)
    sqpnp_transformation = absolute_pose::sqpnp(adapter);
  gettimeofday( &toc, 0 );
  double sqpnp_time = TIMETODOUBLE(timeval_minus(toc,tic)) / iterations;

  std::cout << "results from SQPnP algorithm (pose):" << std::endl;
  std::cout << sqpnp_transformation << std::endl << std::endl;
  std::cout << "computation time: " << sqpnp_time << " seconds" << std::endl;

  // Test angular consistency validation
  std::cout << "\nTesting angular consistency validation:" << std::endl;
  double totalAngularError = 0.0;
  int consistentPoints = 0;
  double threshold = 0.1; // 0.1 radians threshold
  
  for(size_t i = 0; i < bearingVectors.size(); i++)
  {
    double angError = math::angularReprojectionError(
        bearingVectors[i],
        points[i],
        sqpnp_transformation.block<3,3>(0,0),
        sqpnp_transformation.col(3));
    totalAngularError += angError;
    
    if(angError < threshold)
      consistentPoints++;
  }
  
  std::cout << "Average angular error: " << (totalAngularError / bearingVectors.size()) << " radians" << std::endl;
  std::cout << "Points passing angular consistency: " << consistentPoints << "/" << bearingVectors.size() << std::endl;
  
  // Test with backward-facing bearings (omnidirectional)
  std::cout << "\nTesting with backward-facing bearings (omnidirectional):" << std::endl;
  bearingVectors_t omniDirectionalBearings = bearingVectors;
  
  // Flip some bearings to simulate omnidirectional camera
  for(size_t i = 0; i < omniDirectionalBearings.size() / 2; i++)
  {
    omniDirectionalBearings[i] = -omniDirectionalBearings[i];
  }
  
  absolute_pose::CentralAbsoluteAdapter omniAdapter(
      omniDirectionalBearings,
      points,
      rotation );
  
  transformation_t omni_sqpnp_transformation = absolute_pose::sqpnp(omniAdapter);
  
  std::cout << "SQPnP with omnidirectional bearings:" << std::endl;
  std::cout << omni_sqpnp_transformation << std::endl;
  
  std::cout << "\nTest completed successfully!" << std::endl;
  
  return 0;
}
