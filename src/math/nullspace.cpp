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

#include <opengv/math/nullspace.hpp>

Eigen::MatrixXd
opengv::math::computeNullSpace(
    const Eigen::MatrixXd & A,
    double tolerance)
{
  // Use SVD to compute null-space
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
  
  // Count how many singular values are effectively zero
  int nullDim = 0;
  for(int i = 0; i < svd.singularValues().size(); i++)
  {
    if(svd.singularValues()(i) < tolerance)
      nullDim++;
  }
  
  // Extract null-space basis from right singular vectors
  if(nullDim == 0)
    return Eigen::MatrixXd(A.cols(), 0);
  
  Eigen::MatrixXd nullBasis = svd.matrixV().rightCols(nullDim);
  return nullBasis;
}

Eigen::MatrixXd
opengv::math::gramSchmidt(const Eigen::MatrixXd & vectors)
{
  if(vectors.cols() == 0)
    return vectors;
  
  Eigen::MatrixXd ortho = vectors;
  
  // Apply Gram-Schmidt orthogonalization
  for(int i = 0; i < ortho.cols(); i++)
  {
    // Normalize current vector
    ortho.col(i).normalize();
    
    // Orthogonalize remaining vectors against current one
    for(int j = i + 1; j < ortho.cols(); j++)
    {
      double projection = ortho.col(i).dot(ortho.col(j));
      ortho.col(j) -= projection * ortho.col(i);
    }
  }
  
  // Final normalization pass
  for(int i = 0; i < ortho.cols(); i++)
    ortho.col(i).normalize();
  
  return ortho;
}

Eigen::VectorXd
opengv::math::projectToNullSpace(
    const Eigen::VectorXd & v,
    const Eigen::MatrixXd & nullBasis)
{
  // Project v onto the null-space spanned by nullBasis columns
  Eigen::VectorXd projected = Eigen::VectorXd::Zero(v.size());
  
  for(int i = 0; i < nullBasis.cols(); i++)
  {
    double coeff = nullBasis.col(i).dot(v);
    projected += coeff * nullBasis.col(i);
  }
  
  return projected;
}
