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
 * \file nullspace.hpp
 * \brief Functions for null-space computation and orthogonalization,
 *        inspired by SQPnP approach for robust pose estimation.
 */

#ifndef OPENGV_NULLSPACE_HPP_
#define OPENGV_NULLSPACE_HPP_

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
 * \brief Compute the null-space basis of a matrix using Gram-Schmidt orthogonalization.
 *        This provides a numerically stable way to compute orthonormal basis vectors.
 *
 * \param[in] A Input matrix for which to compute null-space.
 * \param[in] tolerance Threshold for considering singular values as zero.
 * \return Matrix whose columns form an orthonormal basis for the null-space.
 */
Eigen::MatrixXd computeNullSpace(
    const Eigen::MatrixXd & A,
    double tolerance = 1e-10);

/**
 * \brief Apply Gram-Schmidt orthogonalization to a set of vectors.
 *
 * \param[in] vectors Matrix whose columns are the vectors to orthogonalize.
 * \return Matrix with orthonormalized column vectors.
 */
Eigen::MatrixXd gramSchmidt(const Eigen::MatrixXd & vectors);

/**
 * \brief Project a vector onto the null-space of a matrix.
 *
 * \param[in] v Vector to project.
 * \param[in] nullBasis Null-space basis matrix (columns are basis vectors).
 * \return Projected vector in the null-space.
 */
Eigen::VectorXd projectToNullSpace(
    const Eigen::VectorXd & v,
    const Eigen::MatrixXd & nullBasis);

}
}

#endif /* OPENGV_NULLSPACE_HPP_ */
