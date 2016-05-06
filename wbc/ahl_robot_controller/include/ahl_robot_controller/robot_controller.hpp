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

#ifndef __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP
#define __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP

#include <map>
#include <list>
#include <memory>
#include <ahl_robot/ahl_robot.hpp>
#include "ahl_robot_controller/param_base.hpp"
#include "ahl_robot_controller/task/task.hpp"
#include "ahl_robot_controller/task/multi_task.hpp"

namespace ahl_ctrl
{

  class RobotController
  {
  public:
    explicit RobotController();

    void init(const ahl_robot::RobotPtr& robot);
    void init(const ahl_robot::RobotPtr& robot, const ParamBasePtr& param);
    void addTask(const TaskPtr& task, int32_t priority);
    void clearTask();
    void updateModel();
    void simulate(double period, const Eigen::VectorXd& tau, Eigen::VectorXd& qd, Eigen::VectorXd& dqd, Eigen::VectorXd& ddqd);
    void computeGeneralizedForce(Eigen::VectorXd& tau);

  private:
    ParamBasePtr param_;
    MultiTaskPtr multi_task_;
    ahl_robot::RobotPtr robot_;
    std::vector<ahl_robot::ManipulatorPtr> mnp_;
    uint32_t dof_;
  };

  using RobotControllerPtr = std::shared_ptr<RobotController>;

} // namespace ahl_ctrl

#endif // __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP
