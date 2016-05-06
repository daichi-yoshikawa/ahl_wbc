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

#include "ahl_robot_controller/task/joint_limit.hpp"

using namespace ahl_ctrl;

JointLimit::JointLimit(const ahl_robot::ManipulatorPtr& mnp, double threshold)
  : threshold_(threshold)
{
  mnp_ = mnp;
  tau_ = Eigen::VectorXd::Zero(mnp_->getLinkNum());

  q_max_ = Eigen::VectorXd::Zero(mnp_->getLinkNum());
  q_min_ = Eigen::VectorXd::Zero(mnp_->getLinkNum());

  lock_.resize(mnp_->getLinkNum());

  for(uint32_t i = mnp_->getMacroManipulatorDOF(); i < mnp_->getLinkNum(); ++i)
  {
    q_max_[i] = mnp_->getLink(i)->q_max;
    q_min_[i] = mnp_->getLink(i)->q_min;
    lock_[i]  = false;
  }

  N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
}

void JointLimit::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(mnp_->getDOF());
  N_  = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());

  for(uint32_t i = mnp_->getMacroManipulatorDOF(); i < mnp_->q().rows(); ++i)
  {
    if(q_min_[i] == q_max_[i]) continue;

    double q = mnp_->q()[i];
    double max = q_max_[i];
    double min = q_min_[i];

    Eigen::VectorXd tau_damp = Eigen::VectorXd::Zero(mnp_->getDOF());
    tau_damp = -mnp_->getMassMatrix() * param_->getKvDamp().coeff(0, 0) * mnp_->dq();

    if(lock_[i])
    {
      if(moveAwayFromMax(mnp_->q()[i], q_max_[i]) &&
         moveAwayFromMin(mnp_->q()[i], q_min_[i]))
      {
        lock_[i] = false;
        N_.coeffRef(i, i) = 1.0;
      }
    }

    if(max - threshold_ < q && q < max)
    {
      //std::cout << i << " locked" << std::endl;
      tau[i] = -param_->getKpLimit().coeff(i, i) * (max - q) + tau_damp[i];
      N_.coeffRef(i, i) = 0.0;
      lock_[i] = true;
    }
    else if(max <= q)
    {
      tau[i] = -param_->getKpLimit().coeff(i, i) * threshold_ + tau_damp[i];
      N_.coeffRef(i, i) = 0.0;
      lock_[i] = true;
    }
    else if(min < q < min + threshold_)
    {
      tau[i] = -param_->getKpLimit().coeff(i, i) * (min - q) + tau_damp[i];
      N_.coeffRef(i, i) = 0.0;
      lock_[i] = true;
    }
    else if(q <= min)
    {
      tau[i] = param_->getKpLimit().coeff(i, i) * threshold_ + tau_damp[i];
      N_.coeffRef(i, i) = 0.0;
      lock_[i] = true;
    }
  }
}

bool JointLimit::moveAwayFromMax(double q, double max)
{
  if(q < max - 2.0 * threshold_) return true;
  return false;
}

bool JointLimit::moveAwayFromMin(double q, double min)
{
  if(q > min + 2.0 * threshold_) return true;
  return false;
}
