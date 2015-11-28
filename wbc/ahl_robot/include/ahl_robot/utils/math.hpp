/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

/////////////////////////////////////////////////////
/// \file math.hpp
/// \brief Declare mathematical method
/// \author Daichi Yoshikawa
/////////////////////////////////////////////////////

#ifndef __AHL_ROBOT_MATH_HPP
#define __AHL_ROBOT_MATH_HPP

#include <vector>
#include <Eigen/Dense>

namespace ahl_robot
{
  namespace math
  {
    /// Compute matrix which convert from quaternion to 3 dimensional vector
    /// \param q Quaternion
    /// \param Er Conversion matrix
    void computeEr(const Eigen::Quaternion<double>& q, Eigen::MatrixXd& Er);
    void calculateInverseTransformationMatrix(const Eigen::Matrix4d& src, Eigen::Matrix4d& dst);
    void rpyToRotationMatrix(const std::vector<double>& rpy, Eigen::Matrix3d& mat);
    void rpyToRotationMatrix(const Eigen::Vector3d& rpy, Eigen::Matrix3d& mat);
    void rpyToQuaternion(const Eigen::Vector3d& rpy, Eigen::Quaternion<double>& q);
    void xyzrpyToTransformationMatrix(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy, Eigen::Matrix4d& T);
  }
}

#endif /* __AHL_ROBOT_MATH_HPP */
