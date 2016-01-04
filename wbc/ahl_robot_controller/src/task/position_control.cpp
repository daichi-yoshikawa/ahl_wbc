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
#include "ahl_robot_controller/task/position_control.hpp"

using namespace ahl_ctrl;

PositionControl::PositionControl(const ahl_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh)
  : updated_(false), target_link_(target_link), eigen_thresh_(eigen_thresh)
{
  mnp_ = mnp;

  if(!mnp_->hasLink(target_link))
  {
    std::stringstream msg;
    msg << target_link << " was not found in mnp_->name_to_idx.";
    throw ahl_utils::Exception("PositionControl::PositionControl", msg.str()); 
  }

  idx_ = mnp_->getIndex(target_link);
  I_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
  N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
  F_unit_ = Eigen::Vector3d::Zero();
  error_sum_ = Eigen::Vector3d::Zero();
}

void PositionControl::setGoal(const Eigen::MatrixXd& xd)
{
  if(xd.rows() != 3)
  {
    std::stringstream msg;
    msg << "xd.rows() != 3" << std::endl
        << "  xd.rows : " << xd.rows();
    throw ahl_utils::Exception("PositionControl::setGoal", msg.str());
  }

  xd_ = xd.block(0, 0, xd.rows(), 1);
}

void PositionControl::updateModel()
{
  Jv_ = mnp_->getJacobian()[idx_].block(0, 0, 3, mnp_->getJacobian()[idx_].cols());
  lambda_inv_ = Jv_ * mnp_->getMassMatrixInv() * Jv_.transpose();
  EffectiveMassMatrix3d::compute(lambda_inv_, lambda_, eigen_thresh_);
  J_dyn_inv_ = mnp_->getMassMatrixInv() * Jv_.transpose() * lambda_;
  N_ = I_ - Jv_.transpose() * J_dyn_inv_.transpose();
  updated_ = true;
}

void PositionControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  if(!updated_)
  {
    tau = Eigen::VectorXd::Zero(mnp_->getDOF());
    return;
  }

  Eigen::Vector3d x = mnp_->getTransformAbs(idx_).block(0, 3, 3, 1);
  Eigen::Vector3d error = xd_ - x;

/*  if(error.norm() > param_->getPosErrorMax())
  {
    error = param_->getPosErrorMax() / error.norm() * error;
  }
*/
  Eigen::Matrix3d Kpv = param_->getKpTask().block(0, 0, 3, 3);

  for(unsigned int i = 0; i < Kpv.rows(); ++i)
  {
    Kpv.coeffRef(i, i) /= param_->getKvTask().block(0, 0, 3, 3).coeff(i, i);
  }

  error_sum_ += error;
  for(unsigned int i = 0; i < error_sum_.rows(); ++i)
  {
    if(error_sum_[i] > param_->getIClippingTaskPos()[i])
      error_sum_[i] = param_->getIClippingTaskPos()[i];
    else if(error_sum_[i] < -param_->getIClippingTaskPos()[i])
      error_sum_[i] = -param_->getIClippingTaskPos()[i];
  }

  Eigen::Vector3d dxd = Kpv * error + param_->getKiTask().block(0, 0, 3, 3) * error_sum_;

  if(dxd.norm() > param_->getVxMax())
  {
    dxd = param_->getVxMax() / dxd.norm() * dxd;
  }

  F_unit_ = -param_->getKvTask().block(0, 0, 3, 3) * (Jv_ * mnp_->dq() - dxd);
  Eigen::Vector3d F = lambda_ * F_unit_;
  tau = tau_ = Jv_.transpose() * F;
}
