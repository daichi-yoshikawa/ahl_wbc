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

#include "ahl_utils/exception.hpp"
#include "ahl_robot_controller/common/effective_mass_matrix3d.hpp"
#include "ahl_robot_controller/task/orientation_control.hpp"

using namespace ahl_ctrl;

OrientationControl::OrientationControl(const ahl_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh)
  : updated_(false), target_link_(target_link), eigen_thresh_(eigen_thresh)
{
  mnp_ = mnp;

  if(!mnp_->hasLink(target_link))
  {
    std::stringstream msg;
    msg << target_link << " was not found in mnp_->name_to_idx.";
    throw ahl_utils::Exception("OrientationControl::OrientationControl", msg.str());
  }

  idx_ = mnp_->getIndex(target_link);
  I_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
  N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
  M_unit_ = Eigen::Vector3d::Zero();
}

void OrientationControl::setGoal(const Eigen::MatrixXd& Rd)
{
  if(Rd.rows() != 3)
  {
    std::stringstream msg;
    msg << "Rd.rows() != 3" << std::endl
        << "  Rd.rows : " << Rd.rows();
    throw ahl_utils::Exception("OrientationControl::setGoal", msg.str());
  }
  if(Rd.cols() != 3)
  {
    std::stringstream msg;
    msg << "Rd.cols() != 3" << std::endl
        << "  Rd.cols : " << Rd.cols();
    throw ahl_utils::Exception("OrientationControl::setGoal", msg.str());
  }

  Rd_ = Rd;
}

void OrientationControl::updateModel()
{
  Jw_ = mnp_->getJacobian()[idx_].block(3, 0, 3, mnp_->getJacobian()[idx_].cols());
  lambda_inv_ = Jw_ * mnp_->getMassMatrixInv() * Jw_.transpose();
  EffectiveMassMatrix3d::compute(lambda_inv_, lambda_, eigen_thresh_);
  J_dyn_inv_ = mnp_->getMassMatrixInv() * Jw_.transpose() * lambda_;
  N_ = I_ - Jw_.transpose() * J_dyn_inv_.transpose();

  updated_ = true;
}

void OrientationControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  if(!updated_)
  {
    tau = Eigen::VectorXd::Zero(mnp_->getDOF());
    return;
  }

  Eigen::Matrix3d R = mnp_->getTransformAbs(idx_).block(0, 0, 3, 3);
  Eigen::Quaternion<double> q;
  q = R * Rd_.inverse();
  double norm = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  Eigen::Vector3d del_phi;
  double c = 0.0;
  if(norm != 0.0)
  {
    c = 2.0 * acos(q.w()) / norm;

    if(c > param_->getOriErrorMax())
      c = param_->getOriErrorMax();
    else if(c < -param_->getOriErrorMax())
      c = -param_->getOriErrorMax();
  }
  del_phi << q.x() * c, q.y() * c, q.z() * c;

  M_unit_ = -param_->getKpTask().block(3, 3, 3, 3) * del_phi -param_->getKvTask().block(3, 3, 3, 3) * Jw_ * mnp_->dq();
  Eigen::Vector3d M = lambda_ * M_unit_;
  tau = tau_ = Jw_.transpose() * M;
}
