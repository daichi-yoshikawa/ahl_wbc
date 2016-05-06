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

#include "ahl_robot/definition.hpp"
#include "ahl_robot_controller/robot_controller.hpp"
#include "ahl_robot_controller/param.hpp"

using namespace ahl_ctrl;

RobotController::RobotController()
  : dof_(0)
{
}

void RobotController::init(const ahl_robot::RobotPtr& robot)
{
  robot_ = robot;
  param_ = std::make_shared<Param>(robot_);

  for(unsigned int i = 0; i < robot->getManipulatorName().size(); ++i)
  {
    mnp_.push_back(robot->getManipulator(robot->getManipulatorName()[i]));
  }

  dof_ = robot_->getDOF();
  multi_task_ = std::make_shared<MultiTask>(robot_);
}

void RobotController::init(const ahl_robot::RobotPtr& robot, const ParamBasePtr& param)
{
  robot_ = robot;
  param_ = param;

  for(unsigned int i = 0; i < robot->getManipulatorName().size(); ++i)
  {
    mnp_.push_back(robot->getManipulator(robot->getManipulatorName()[i]));
  }

  dof_ = robot_->getDOF();
  multi_task_ = std::make_shared<MultiTask>(robot_);
}

void RobotController::addTask(const TaskPtr& task, int priority)
{
  task->setParam(param_);
  multi_task_->addTask(task, priority);
}

void RobotController::clearTask()
{
  multi_task_->clear();
}

void RobotController::updateModel()
{
  multi_task_->updateModel();
}

void RobotController::simulate(double period, const Eigen::VectorXd& tau, Eigen::VectorXd& qd, Eigen::VectorXd& dqd, Eigen::VectorXd& ddqd)
{
  Eigen::MatrixXd M_inv = robot_->getMassMatrixInv();
  unsigned int macro_dof = robot_->getMacroManipulatorDOF();

  Eigen::VectorXd g = Eigen::VectorXd::Zero(robot_->getDOF());
  if(multi_task_->haveGravityCompensation())
  {
    g = multi_task_->getGravityCompensation();
  }

  ddqd = M_inv * (tau - g);
  dqd = robot_->getJointVelocity() + period * ddqd;
  qd = robot_->getJointPosition() + dqd * period + 0.5 * ddqd * period * period;
}

void RobotController::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  multi_task_->computeGeneralizedForce(tau);

  unsigned int macro_dof = robot_->getMacroManipulatorDOF();

  for(unsigned int i = 0; i < macro_dof; ++i)
  {
    if(tau[i] > (*mnp_.begin())->getLink(i)->tau_max)
    {
      tau[i] = (*mnp_.begin())->getLink(i)->tau_max;
    }
    else if(tau[i] < -(*mnp_.begin())->getLink(i)->tau_max)
    {
      tau[i] = -(*mnp_.begin())->getLink(i)->tau_max;
    }
  }

  unsigned int idx_offset = macro_dof;

  for(unsigned int i = 0; i < mnp_.size(); ++i)
  {
    unsigned int mini_dof = mnp_[i]->getDOF() - macro_dof;

    for(unsigned int j = 0; j < mini_dof; ++j)
    {
      if(tau[j + idx_offset] > mnp_[i]->getLink(j + macro_dof)->tau_max)
      {
        tau[j + idx_offset] = mnp_[i]->getLink(j + macro_dof)->tau_max;
      }
      else if(tau[j + idx_offset] < -mnp_[i]->getLink(j + macro_dof)->tau_max)
      {
        tau[j + idx_offset] = -mnp_[i]->getLink(j + macro_dof)->tau_max;
      }
    }

    idx_offset += mini_dof;
  }
}
