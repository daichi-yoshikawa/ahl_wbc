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

#include "ahl_robot_controller/task/friction_compensation.hpp"

using namespace ahl_ctrl;

FrictionCompensation::FrictionCompensation(const ahl_robot::RobotPtr& robot)
{
  robot_ = robot;
  mnp_name_ = robot_->getManipulatorName();
  N_ = Eigen::MatrixXd::Identity(robot->getDOF(), robot->getDOF());
  b_ = Eigen::VectorXd::Zero(robot->getDOF()).asDiagonal();
}

void FrictionCompensation::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(robot_->getDOF());
  unsigned int macro_dof = robot_->getMacroManipulatorDOF();

  unsigned int offset = 0;
  for(unsigned int i = 0; i < mnp_name_.size(); ++i)
  {
    mnp_ = robot_->getManipulator(mnp_name_[i]);
    unsigned int mini_dof = mnp_->getDOF() - macro_dof;

    tau.block(macro_dof + offset, 0, mini_dof, 1) = b_.block(macro_dof + offset, macro_dof + offset, mini_dof, mini_dof) * mnp_->dq().block(macro_dof, 0, mini_dof, 1);

    offset += mini_dof;
  }

  tau_ = tau;
}
