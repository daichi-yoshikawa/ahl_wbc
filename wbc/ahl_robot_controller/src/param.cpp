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

#include "ahl_robot_controller/param.hpp"

using namespace ahl_ctrl;

Param::Param(const ahl_robot::RobotPtr& robot)
  : dof_(robot->getDOF()),
    macro_dof_(robot->getMacroManipulatorDOF()),
    joint_error_max_(100.0), pos_error_max_(100.0), ori_error_max_(100.0),
    dq_max_(1.0), vx_max_(0.5),
    kp_wheel_(0.0), kv_wheel_(0.0)
{
  Kp_joint_ = Eigen::MatrixXd::Zero(dof_, dof_);
  Kv_joint_ = Eigen::MatrixXd::Zero(dof_, dof_);
  Kp_task_  = Eigen::MatrixXd::Zero(6, 6);
  Ki_task_  = Eigen::MatrixXd::Zero(6, 6);
  Kv_task_  = Eigen::MatrixXd::Zero(6, 6);
  i_clipping_task_pos_ = Eigen::Vector3d::Zero();
  i_clipping_task_ori_ = Eigen::Vector3d::Zero();
  Kv_damp_  = Eigen::MatrixXd::Zero(dof_, dof_);
  Kp_limit_ = Eigen::MatrixXd::Zero(dof_, dof_);
  Kv_limit_ = Eigen::MatrixXd::Zero(dof_, dof_);

  g_ << 0.0, 0.0, -9.80665;
  b_ = Eigen::MatrixXd::Zero(dof_, dof_);
  ros::NodeHandle local_nh("~/ahl_robot_controller");
  f_ = boost::bind(&Param::update, this, _1, _2);
  server_ = ParamConfigServerPtr(new ParamConfigServer(local_nh));
  server_->setCallback(f_);
}

void Param::update(ahl_robot_controller::ParamConfig& config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  Eigen::VectorXd Kp_joint = Eigen::VectorXd::Constant(dof_, config.kp_joint);
  Eigen::VectorXd Kv_joint = Eigen::VectorXd::Constant(dof_, config.kv_joint);
  Kp_joint.block(0, 0, macro_dof_, 1) = Eigen::VectorXd::Constant(macro_dof_, config.kp_joint_macro);
  Kv_joint.block(0, 0, macro_dof_, 1) = Eigen::VectorXd::Constant(macro_dof_, config.kv_joint_macro);
  Eigen::VectorXd Kp_task_pos = Eigen::VectorXd::Constant(3, config.kp_task_pos);
  Eigen::VectorXd Ki_task_pos = Eigen::VectorXd::Constant(3, config.ki_task_pos);
  Eigen::VectorXd Kv_task_pos = Eigen::VectorXd::Constant(3, config.kv_task_pos);
  Eigen::VectorXd Kp_task_ori = Eigen::VectorXd::Constant(3, config.kp_task_ori);
  Eigen::VectorXd Ki_task_ori = Eigen::VectorXd::Constant(3, config.ki_task_ori);
  Eigen::VectorXd Kv_task_ori = Eigen::VectorXd::Constant(3, config.kv_task_ori);
  Eigen::VectorXd Kv_damp  = Eigen::VectorXd::Constant(dof_, config.kv_damp);
  Eigen::VectorXd Kp_limit = Eigen::VectorXd::Constant(dof_, config.kp_limit);
  Eigen::VectorXd Kv_limit = Eigen::VectorXd::Constant(dof_, config.kv_limit);

  Kp_joint_ = Kp_joint.asDiagonal();
  Kv_joint_ = Kv_joint.asDiagonal();
  Kp_task_.block(0, 0, 3, 3) = Kp_task_pos.asDiagonal();
  Ki_task_.block(0, 0, 3, 3) = Ki_task_pos.asDiagonal();
  Kv_task_.block(0, 0, 3, 3) = Kv_task_pos.asDiagonal();

  Kp_task_.block(3, 3, 3, 3) = Kp_task_ori.asDiagonal();
  Ki_task_.block(3, 3, 3, 3) = Ki_task_ori.asDiagonal();
  Kv_task_.block(3, 3, 3, 3) = Kv_task_ori.asDiagonal();
  Kv_damp_  = Kv_damp.asDiagonal();
  Kp_limit_ = Kp_limit.asDiagonal();
  Kv_limit_ = Kv_limit.asDiagonal();

  joint_error_max_ = config.joint_error_max;
  pos_error_max_ = config.pos_error_max;
  ori_error_max_ = config.ori_error_max;
  dq_max_ = config.dq_max;
  vx_max_ = config.vx_max;

  g_ << config.gx, config.gy, config.gz;
  b_ = Eigen::VectorXd::Constant(dof_, config.b).asDiagonal();

  i_clipping_task_pos_ = Eigen::Vector3d::Constant(config.i_clipping_task_pos);
  i_clipping_task_ori_ = Eigen::Vector3d::Constant(config.i_clipping_task_ori);

  kp_wheel_ = config.kp_wheel;
  kv_wheel_ = config.kv_wheel;
}
