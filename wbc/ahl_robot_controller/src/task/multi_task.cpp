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
#include "ahl_robot_controller/task/multi_task.hpp"

using namespace ahl_ctrl;

MultiTask::MultiTask(const ahl_robot::RobotPtr& robot)
{
  dof_ = robot->getDOF();
  macro_dof_ = robot->getMacroManipulatorDOF();

  std::vector<std::string> name = robot->getManipulatorName();

  for(uint32_t i = 0; i < name.size(); ++i)
  {
    name_to_mini_dof_[name[i]] = robot->getDOF(name[i]) - macro_dof_;

    if(i > 0)
    {
      name_to_offset_[name[i]] = name_to_offset_[name[i - 1]] + name_to_mini_dof_[name[i - 1]];
    }
    else
    {
      name_to_offset_[name[i]] = 0;
    }
  }

  name_to_mini_dof_[robot->getName()] = dof_ - macro_dof_;
  name_to_offset_[robot->getName()] = 0;
}

void MultiTask::addTask(const TaskPtr& task, int32_t priority)
{
  if(multi_task_.find(priority) != multi_task_.end())
  {
    if(task->haveNullSpace())
    {
      for(uint32_t i = 0; i < multi_task_[priority].size(); ++i)
      {
        if(multi_task_[priority][i]->haveNullSpace())
        {
          std::stringstream msg;
          msg << "Two tasks with the same priority have null spaces." << std::endl
              << "  priority : " << priority;
          throw ahl_utils::Exception("MultiTask::addTask", msg.str());
        }
      }
    }
  }

  multi_task_[priority].push_back(task);
  if(task->getName() == ahl_ctrl::task::GRAVITY_COMPENSATION)
  {
    gravity_compensation_ = task;
  }
}

void MultiTask::clear()
{
  multi_task_.clear();
  gravity_compensation_.reset();
}

void MultiTask::updateModel()
{
  std::map<int32_t, std::vector<TaskPtr> >::iterator it;
  for(it = multi_task_.begin(); it != multi_task_.end(); ++it)
  {
    for(uint32_t i = 0; i < it->second.size(); ++i)
    {
      it->second[i]->updateModel();
    }
  }
}

void MultiTask::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(dof_);
  Eigen::VectorXd tmp = Eigen::VectorXd::Zero(dof_);

  std::map<int32_t, std::vector<TaskPtr> >::iterator it = multi_task_.begin();
  for(it = multi_task_.begin(); it != multi_task_.end(); ++it)
  {
    N_ = Eigen::MatrixXd::Identity(dof_, dof_);

    Eigen::VectorXd tau_sum = Eigen::VectorXd::Zero(dof_);

    for(uint32_t i = 0; i < it->second.size(); ++i)
    {
      Eigen::VectorXd tau_task;
      it->second[i]->computeGeneralizedForce(tau_task);

      Eigen::VectorXd extended_tau_task = Eigen::VectorXd::Zero(dof_);
      this->assignTorque(tau_task, extended_tau_task, it->second[i]->getTargetName());
      tau_sum += extended_tau_task;

      if(it->second[i]->haveNullSpace())
      {
        Eigen::MatrixXd N = it->second[i]->getNullSpace();
        this->assignNullSpace(N, N_, it->second[i]->getTargetName());
      }
    }

    tau = N_ * tau;
    tau += tau_sum;
  }
}

bool MultiTask::haveGravityCompensation()
{
  if(gravity_compensation_)
    return true;
  return false;
}

void MultiTask::assignTorque(const Eigen::VectorXd& src, Eigen::VectorXd& dst, const std::string& name)
{
  dst.block(0, 0, macro_dof_, 1) = src.block(0, 0, macro_dof_, 1);
  dst.block(macro_dof_ + name_to_offset_[name], 0, name_to_mini_dof_[name], 1) = src.block(macro_dof_, 0, name_to_mini_dof_[name], 1);
}

void MultiTask::assignNullSpace(const Eigen::MatrixXd& src, Eigen::MatrixXd& dst, const std::string& name)
{
  uint32_t offset   = macro_dof_ + name_to_offset_[name];
  uint32_t mini_dof = name_to_mini_dof_[name];

  dst.block(0, 0, macro_dof_, macro_dof_) = src.block(0, 0, macro_dof_, macro_dof_);
  dst.block(offset, 0, mini_dof, macro_dof_) = src.block(macro_dof_, 0, mini_dof, macro_dof_);
  dst.block(0, offset, macro_dof_, mini_dof) = src.block(0, macro_dof_, macro_dof_, mini_dof);
  dst.block(offset, offset, mini_dof, mini_dof) = src.block(macro_dof_, macro_dof_, mini_dof, mini_dof);
}
