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

#ifndef __AHL_ROBOT_SAMPLES_PR2_HPP
#define __AHL_ROBOT_SAMPLES_PR2_HPP

#include "ahl_robot_samples/robot_demo.hpp"
#include "ahl_robot_samples/pr2/pr2_param.hpp"

namespace ahl_sample
{

  class PR2 : public RobotDemo
  {
  public:
    PR2();

    virtual void init();
    virtual void run();
  private:
    virtual void updateModel(const ros::TimerEvent&);
    virtual void control(const ros::TimerEvent&);

    Pr2ParamPtr param_;

    TaskPtr gravity_compensation_;
    TaskPtr joint_limit_l_;
    TaskPtr joint_limit_r_;
    TaskPtr joint_control_l_;
    TaskPtr joint_control_r_;
    TaskPtr position_control_l_;
    TaskPtr position_control_r_;
    TaskPtr orientation_control_l_;
    TaskPtr orientation_control_r_;
  };

  typedef boost::shared_ptr<PR2> PR2Ptr;
}

#endif /* __AHL_ROBOT_SAMPLES_PR2_HPP */
