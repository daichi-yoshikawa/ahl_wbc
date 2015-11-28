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

#include "ahl_robot_controller/task/gravity_compensation.hpp"

using namespace ahl_ctrl;

GravityCompensation::GravityCompensation(const ahl_robot::RobotPtr& robot)
{
  robot_ = robot;
  mnp_name_ = robot_->getManipulatorName();
  N_ = Eigen::MatrixXd::Identity(robot->getDOF(), robot->getDOF());
}

void GravityCompensation::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(robot_->getDOF());
  unsigned int macro_dof = robot_->getMacroManipulatorDOF();

  mnp_ = robot_->getManipulator().begin()->second;
  for(unsigned int i = 0; i < macro_dof; ++i)
  {
    tau.block(0, 0, macro_dof, 1) -= mnp_->getLink(i)->m * mnp_->getJacobian()[i].block(0, 0, 3, macro_dof).transpose() * param_->getG();
  }

  unsigned int offset = 0;

  for(unsigned int i = 0; i < mnp_name_.size(); ++i)
  {
    mnp_ = robot_->getManipulator(mnp_name_[i]);
    unsigned int mini_dof = mnp_->getDOF() - macro_dof;

    for(unsigned int j = 0; j < mini_dof; ++j)
    {
      tau.block(macro_dof + offset, 0, mini_dof, 1) -= mnp_->getLink(j + macro_dof)->m * mnp_->getJacobian()[j + macro_dof].block(0, macro_dof, 3, mini_dof).transpose() * param_->getG();
    }

    offset += mini_dof;
  }

  tau_ = tau;
}
