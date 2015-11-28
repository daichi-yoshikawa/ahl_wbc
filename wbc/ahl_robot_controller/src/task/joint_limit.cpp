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

  q_max_.resize(mnp_->getLinkNum());
  q_min_.resize(mnp_->getLinkNum());

  for(unsigned int i = 0; i < mnp_->getLinkNum(); ++i)
  {
    q_max_.coeffRef(i) = mnp_->getLink(i)->q_max;
    q_min_.coeffRef(i) = mnp_->getLink(i)->q_min;
  }

  N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
}

void JointLimit::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(mnp_->getDOF());

  N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());

  for(unsigned int i = 0; i < mnp_->q().rows(); ++i)
  {
    double q_max_diff = q_max_.coeff(i) - mnp_->q().coeff(i);
    double q_min_diff = q_min_.coeff(i) - mnp_->q().coeff(i);

    if(q_max_diff < 0.0)
    {
      std::cout << i << " : max : " << mnp_->q().coeff(i) << std::endl;

      q_max_diff = threshold_;
      tau.coeffRef(i) += -param_->getKpLimit().coeff(i, i) * q_max_diff - param_->getKvLimit().coeff(i, i) * mnp_->dq().coeff(i);
      N_.coeffRef(i, i) = 0;

    }
    else if(q_max_diff < threshold_)
    {
      std::cout << i << " : max : " << mnp_->q().coeff(i) << std::endl;

      tau.coeffRef(i) += -param_->getKpLimit().coeff(i, i) * q_max_diff - param_->getKvLimit().coeff(i, i) * mnp_->dq().coeff(i);
      N_.coeffRef(i, i) = 0;
    }

    if(q_min_diff > 0.0)
    {
      std::cout << i << " : min : " << mnp_->q().coeff(i) << std::endl;

      q_min_diff = -threshold_;
      tau.coeffRef(i) += -param_->getKpLimit().coeff(i, i) * q_min_diff - param_->getKvLimit().coeff(i, i) * mnp_->dq().coeff(i);
      N_.coeffRef(i, i) = 0;
    }
    else if(-q_min_diff < threshold_)
    {
      std::cout << i << " : min : " << mnp_->q().coeff(i) << std::endl;
      tau.coeffRef(i) += -param_->getKpLimit().coeff(i, i) * q_min_diff - param_->getKvLimit().coeff(i, i) * mnp_->dq().coeff(i);
      N_.coeffRef(i, i) = 0;
    }
  }
}
