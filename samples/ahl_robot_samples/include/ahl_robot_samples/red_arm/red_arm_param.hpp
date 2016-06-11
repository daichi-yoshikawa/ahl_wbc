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

#ifndef __AHL_ROBOT_SAMPLES_RED_ARM_PARAM_HPP
#define __AHL_ROBOT_SAMPLES_RED_ARM_PARAM_HPP

#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "ahl_robot_samples/RedArmParamConfig.h"

namespace ahl_sample
{

  class RedArmParam
  {
  public:
    explicit RedArmParam()
    {
      x = Eigen::Vector3d::Zero();
      R = Eigen::Matrix3d::Identity();
      q = Eigen::VectorXd::Zero(7);

      ros::NodeHandle local_nh("~/red_arm/");
      f_ = boost::bind(&RedArmParam::update, this, _1, _2);
      server_ = std::make_shared<RedArmParamConfigServer>(local_nh);
      server_->setCallback(f_);
    }

    bool show_target = true;
    bool sin_x = false;
    bool sin_y = false;
    bool sin_z = false;

    Eigen::Vector3d x;
    Eigen::Matrix3d R;
    Eigen::VectorXd q;

  private:
    void update(ahl_robot_samples::RedArmParamConfig& config, uint32_t level)
    {
      show_target = config.show_target;

      sin_x = config.sin_x;
      sin_y = config.sin_y;
      sin_z = config.sin_z;

      x << config.x_arm, config.y_arm, config.z_arm;

      R = Eigen::AngleAxisd(config.roll_arm,  Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(config.pitch_arm, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(config.yaw_arm,   Eigen::Vector3d::UnitZ());

      q << config.q1, config.q2, config.q3, config.q4, config.q5, config.q6, config.q7;
    }

    using RedArmParamConfigServer = dynamic_reconfigure::Server<ahl_robot_samples::RedArmParamConfig>;
    using RedArmParamConfigServerPtr = std::shared_ptr<RedArmParamConfigServer>;

    RedArmParamConfigServerPtr server_;
    dynamic_reconfigure::Server<ahl_robot_samples::RedArmParamConfig>::CallbackType f_;
  };

  using RedArmParamPtr = std::shared_ptr<RedArmParam>;

} // namespace ahl_sample

#endif // __AHL_ROBOT_SAMPLES_RED_ARM_PARAM_HPP
