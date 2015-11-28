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

#include <iostream>
#include "ahl_robot/utils/math.hpp"

namespace ahl_robot
{
  namespace math
  {
    void computeEr(const Eigen::Quaternion<double>& q, Eigen::MatrixXd& Er)
    {
      Er.resize(4, 3);
      Er << -q.x(), -q.y(), -q.z(),
             q.w(),  q.z(), -q.y(),
            -q.z(),  q.w(),  q.x(),
             q.y(), -q.x(),  q.w();
      Er = 0.5 * Er;
    }

    void calculateInverseTransformationMatrix(const Eigen::Matrix4d& src, Eigen::Matrix4d& dst)
    {
      dst = Eigen::Matrix4d::Identity();
      Eigen::Matrix3d R_trans = src.block(0, 0, 3, 3).transpose();
      dst.block(0, 0, 3, 3) = R_trans;
      dst.block(0, 3, 3, 1) = -R_trans * src.block(0, 3, 3, 1);
    }

    void rpyToRotationMatrix(const std::vector<double>& rpy, Eigen::Matrix3d& mat)
    {
      double sin_a = sin(rpy[2]);
      double cos_a = cos(rpy[2]);
      double sin_b = sin(rpy[1]);
      double cos_b = cos(rpy[1]);
      double sin_g = sin(rpy[0]);
      double cos_g = cos(rpy[0]);

      mat.coeffRef(0, 0) = cos_a * cos_b;
      mat.coeffRef(0, 1) = cos_a * sin_b * sin_g - sin_a * cos_g;
      mat.coeffRef(0, 2) = cos_a * sin_b * cos_g + sin_a * sin_g;
      mat.coeffRef(1, 0) = sin_a * cos_b;
      mat.coeffRef(1, 1) = sin_a * sin_b * sin_g + cos_a * cos_g;
      mat.coeffRef(1, 2) = sin_a * sin_b * cos_g - cos_a * sin_g;
      mat.coeffRef(2, 0) = -sin_b;
      mat.coeffRef(2, 1) = cos_b * sin_g;
      mat.coeffRef(2, 2) = cos_b * cos_g;
    }

    void rpyToRotationMatrix(const Eigen::Vector3d& rpy, Eigen::Matrix3d& mat)
    {
      double sin_a = sin(rpy.coeff(2));
      double cos_a = cos(rpy.coeff(2));
      double sin_b = sin(rpy.coeff(1));
      double cos_b = cos(rpy.coeff(1));
      double sin_g = sin(rpy.coeff(0));
      double cos_g = cos(rpy.coeff(0));

      mat.coeffRef(0, 0) = cos_a * cos_b;
      mat.coeffRef(0, 1) = cos_a * sin_b * sin_g - sin_a * cos_g;
      mat.coeffRef(0, 2) = cos_a * sin_b * cos_g + sin_a * sin_g;
      mat.coeffRef(1, 0) = sin_a * cos_b;
      mat.coeffRef(1, 1) = sin_a * sin_b * sin_g + cos_a * cos_g;
      mat.coeffRef(1, 2) = sin_a * sin_b * cos_g - cos_a * sin_g;
      mat.coeffRef(2, 0) = -sin_b;
      mat.coeffRef(2, 1) = cos_b * sin_g;
      mat.coeffRef(2, 2) = cos_b * cos_g;
    }

    void rpyToQuaternion(const Eigen::Vector3d& rpy, Eigen::Quaternion<double>& q)
    {
      double a = rpy.coeff(0);
      double b = rpy.coeff(1);
      double g = rpy.coeff(2);

      double sin_b_half = sin(0.5 * b);
      double cos_b_half = cos(0.5 * b);
      double diff_a_g_half = 0.5 * (a - g);
      double sum_a_g_half = 0.5 * (a + g);

      q.x() = sin_b_half * cos(diff_a_g_half);
      q.y() = sin_b_half * sin(diff_a_g_half);
      q.z() = cos_b_half * sin(sum_a_g_half);
      q.w() = cos_b_half * cos(sum_a_g_half);
    }

    void xyzrpyToTransformationMatrix(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy, Eigen::Matrix4d& T)
    {
      T = Eigen::Matrix4d::Identity();
      Eigen::Matrix3d R;
      rpyToRotationMatrix(rpy, R);
      T.block(0, 0, 3, 3) = R;
      T.block(0, 3, 3, 1) = xyz;
    }
  } // namespace math
} // namespace ahl_robot
