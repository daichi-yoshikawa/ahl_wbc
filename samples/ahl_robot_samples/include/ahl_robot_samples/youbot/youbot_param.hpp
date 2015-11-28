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

#ifndef __AHL_ROBOT_SAMPLES_YOUBOT_PARAM_HPP
#define __AHL_ROBOT_SAMPLES_YOUBOT_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "ahl_robot_samples/YouBotParamConfig.h"

namespace ahl_sample
{

  class YouBotParam
  {
  public:
    YouBotParam()
    {
      const unsigned int dof = 8;

      show_target = true;
      sin_x = false;
      sin_y = false;
      sin_z = false;
      x_arm = Eigen::Vector3d::Zero();
      R_arm = Eigen::Matrix3d::Zero();
      x_base = Eigen::Vector3d::Zero();
      R_base = Eigen::Matrix3d::Zero();
      q = Eigen::VectorXd::Zero(dof);

      ros::NodeHandle local_nh("~/youbot/");
      f_ = boost::bind(&YouBotParam::update, this, _1, _2);
      server_ = YouBotParamConfigServerPtr(new YouBotParamConfigServer(local_nh));
      server_->setCallback(f_);
    }

    bool show_target;
    bool sin_x;
    bool sin_y;
    bool sin_z;
    Eigen::Vector3d x_arm;
    Eigen::Matrix3d R_arm;
    Eigen::Vector3d x_base;
    Eigen::Matrix3d R_base;
    Eigen::VectorXd q;
  private:
    void update(ahl_robot_samples::YouBotParamConfig& config, uint32_t level)
    {
      show_target = config.show_target;

      sin_x = config.sin_x;
      sin_y = config.sin_y;
      sin_z = config.sin_z;

      x_arm[0] = config.x_arm;
      x_arm[1] = config.y_arm;
      x_arm[2] = config.z_arm;

      R_arm = Eigen::AngleAxisd(config.roll_arm,  Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(config.pitch_arm, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(config.yaw_arm,   Eigen::Vector3d::UnitZ());

      x_base[0] = config.x_base;
      x_base[1] = config.y_base;
      x_base[2] = config.z_base;

      R_base = Eigen::AngleAxisd(config.roll_base,  Eigen::Vector3d::UnitX())
             * Eigen::AngleAxisd(config.pitch_base, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(config.yaw_base,   Eigen::Vector3d::UnitZ());

      q[0] = config.q_base1;
      q[1] = config.q_base2;
      q[2] = config.q_base3;
      q[3] = config.q1;
      q[4] = config.q2;
      q[5] = config.q3;
      q[6] = config.q4;
      q[7] = config.q5;
    }

    typedef dynamic_reconfigure::Server<ahl_robot_samples::YouBotParamConfig> YouBotParamConfigServer;
    typedef boost::shared_ptr<YouBotParamConfigServer> YouBotParamConfigServerPtr;

    YouBotParamConfigServerPtr server_;
    dynamic_reconfigure::Server<ahl_robot_samples::YouBotParamConfig>::CallbackType f_;
  };

  typedef boost::shared_ptr<YouBotParam> YouBotParamPtr;
}

#endif /* __AHL_ROBOT_SAMPLES_YOUBOT_PARAM_HPP */
