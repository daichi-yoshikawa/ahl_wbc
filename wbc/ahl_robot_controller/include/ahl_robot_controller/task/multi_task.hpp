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

#ifndef __AHL_ROBOT_CONTROLLER_MULTI_TASK_HPP
#define __AHL_ROBOT_CONTROLLER_MULTI_TASK_HPP

#include <map>
#include <memory>
#include "ahl_robot_controller/task/task.hpp"
#include "ahl_robot_controller/task/gravity_compensation.hpp"

namespace ahl_ctrl
{

  class MultiTask
  {
  public:
    explicit MultiTask(const ahl_robot::RobotPtr& robot);
    void addTask(const TaskPtr& task, int32_t priority);
    void clear();
    void updateModel();
    void computeGeneralizedForce(Eigen::VectorXd& tau);
    bool haveGravityCompensation();
    const Eigen::VectorXd& getGravityCompensation() const { return gravity_compensation_->getTorque(); }

  private:
    void assignTorque(const Eigen::VectorXd& src, Eigen::VectorXd& dst, const std::string& name);
    void assignNullSpace(const Eigen::MatrixXd& src, Eigen::MatrixXd& dst, const std::string& name);

    uint32_t dof_;
    uint32_t macro_dof_;
    std::map<std::string, uint32_t> name_to_mini_dof_;
    std::map<std::string, uint32_t> name_to_offset_;

    std::map<int32_t, std::vector<TaskPtr> > multi_task_; // key : priority
    TaskPtr gravity_compensation_;
    Eigen::MatrixXd N_;
  };

  using MultiTaskPtr = std::shared_ptr<MultiTask>;

} // namespace ahl_ctrl

#endif // __AHL_ROBOT_CONTROLLER_MULTI_TASK_HPP
