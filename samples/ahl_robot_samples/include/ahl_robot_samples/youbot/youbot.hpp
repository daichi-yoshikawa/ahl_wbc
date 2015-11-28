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

#ifndef __AHL_ROBOT_SAMPLES_YOUBOT_HPP
#define __AHL_ROBOT_SAMPLES_YOUBOT_HPP

#include "ahl_robot_samples/robot_demo.hpp"
#include "ahl_robot_samples/youbot/youbot_param.hpp"

namespace ahl_sample
{

  class YouBot : public RobotDemo
  {
  public:
    YouBot();

    virtual void init();
    virtual void run();
  private:
    virtual void updateModel(const ros::TimerEvent&);
    virtual void control(const ros::TimerEvent&);
    void updateWheels(const ros::TimerEvent&);

    YouBotParamPtr param_;

    TaskPtr gravity_compensation_;
    TaskPtr joint_control_;
    TaskPtr arm_position_control_;
    TaskPtr arm_orientation_control_;
    TaskPtr base_position_control_;
    TaskPtr base_orientation_control_;
    GazeboInterfacePtr gazebo_interface_wheel_;

    ros::Timer timer_update_wheels_;
    Eigen::VectorXd q_base_;
    Eigen::VectorXd tau_base_;
  };

  typedef boost::shared_ptr<YouBot> YouBotPtr;
}

#endif /* __AHL_ROBOT_SAMPLES_YOUBOT_HPP */
