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

#ifndef __AHL_ROBOT_SAMPLES_PR2_PARAM_HPP
#define __AHL_ROBOT_SAMPLES_PR2_PARAM_HPP

#include <memory>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "ahl_robot_samples/Pr2ParamConfig.h"

namespace ahl_sample
{

  class Pr2Param
  {
  public:
    explicit Pr2Param()
    {
      const uint32_t dof_l = 11;
      const uint32_t dof_r = 11;

      show_target = true;

      sin_x_l = false;
      sin_y_l = false;
      sin_z_l = false;
      sin_x_r = false;
      sin_y_r = false;
      sin_z_r = false;

      xl = Eigen::Vector3d::Zero();
      rl = Eigen::Vector3d::Zero();
      Rl = Eigen::Matrix3d::Identity();
      Rf_l = Eigen::Matrix3d::Identity();
      Rm_l = Eigen::Matrix3d::Identity();
      fl = Eigen::Vector3d::Zero();
      ml = Eigen::Vector3d::Zero();

      xr = Eigen::Vector3d::Zero();
      rr = Eigen::Vector3d::Zero();
      Rr = Eigen::Matrix3d::Identity();
      Rf_r = Eigen::Matrix3d::Identity();
      Rm_r = Eigen::Matrix3d::Identity();
      fr = Eigen::Vector3d::Zero();
      mr = Eigen::Vector3d::Zero();

      q_l = Eigen::VectorXd::Zero(dof_l);
      q_r = Eigen::VectorXd::Zero(dof_r);

      ros::NodeHandle local_nh("~/pr2/");
      f_ = boost::bind(&Pr2Param::update, this, _1, _2);
      server_ = std::make_shared<Pr2ParamConfigServer>(local_nh);
      server_->setCallback(f_);
    }

    bool show_target;

    bool sin_x_l;
    bool sin_y_l;
    bool sin_z_l;
    bool sin_x_r;
    bool sin_y_r;
    bool sin_z_r;

    Eigen::Vector3d xl;
    Eigen::Vector3d rl;
    Eigen::Matrix3d Rl;
    Eigen::Matrix3d Rf_l;
    Eigen::Matrix3d Rm_l;
    Eigen::Vector3d fl; // force
    Eigen::Vector3d ml; // moment

    Eigen::Vector3d xr;
    Eigen::Vector3d rr;
    Eigen::Matrix3d Rr;
    Eigen::Matrix3d Rf_r;
    Eigen::Matrix3d Rm_r;
    Eigen::Vector3d fr; // force
    Eigen::Vector3d mr; // moment

    Eigen::VectorXd q_l;
    Eigen::VectorXd q_r;

  private:
    void update(ahl_robot_samples::Pr2ParamConfig& config, uint32_t level)
    {
      show_target = config.show_target;

      sin_x_l = config.sin_x_l;
      sin_y_l = config.sin_y_l;
      sin_z_l = config.sin_z_l;

      sin_x_r = config.sin_x_r;
      sin_y_r = config.sin_y_r;
      sin_z_r = config.sin_z_r;

      xl[0] = config.x_l;
      xl[1] = config.y_l;
      xl[2] = config.z_l;

      rl[0] = config.roll_l;
      rl[1] = config.pitch_l;
      rl[2] = config.yaw_l;

      Rl = Eigen::AngleAxisd(config.roll_l,  Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(config.pitch_l, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(config.yaw_l,   Eigen::Vector3d::UnitZ());

      Rf_l = Eigen::AngleAxisd(config.Rf_roll_l,  Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(config.Rf_pitch_l, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(config.Rf_yaw_l,   Eigen::Vector3d::UnitZ());

      Rm_l = Eigen::AngleAxisd(config.Rm_roll_l,  Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(config.Rm_pitch_l, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(config.Rm_yaw_l,   Eigen::Vector3d::UnitZ());

      fl[0] = config.fx_l;
      fl[1] = config.fy_l;
      fl[2] = config.fz_l;

      ml[0] = config.mx_l;
      ml[1] = config.my_l;
      ml[2] = config.mz_l;

      xr[0] = config.x_r;
      xr[1] = config.y_r;
      xr[2] = config.z_r;

      rr[0] = config.roll_r;
      rr[1] = config.pitch_r;
      rr[2] = config.yaw_r;

      Rr = Eigen::AngleAxisd(config.roll_r,  Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(config.pitch_r, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(config.yaw_r,   Eigen::Vector3d::UnitZ());

      Rf_r = Eigen::AngleAxisd(config.Rf_roll_r,  Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(config.Rf_pitch_r, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(config.Rf_yaw_r,   Eigen::Vector3d::UnitZ());

      Rm_r = Eigen::AngleAxisd(config.Rm_roll_r,  Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(config.Rm_pitch_r, Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(config.Rm_yaw_r,   Eigen::Vector3d::UnitZ());

      fr[0] = config.fx_r;
      fr[1] = config.fy_r;
      fr[2] = config.fz_r;

      mr[0] = config.mx_r;
      mr[1] = config.my_r;
      mr[2] = config.mz_r;

      q_l[0]  = config.q_base1;
      q_l[1]  = config.q_base2;
      q_l[2]  = config.q_base3;
      q_l[3]  = config.q_base4;
      q_l[4]  = config.q_l1;
      q_l[5]  = config.q_l2;
      q_l[6]  = config.q_l3;
      q_l[7]  = config.q_l4;
      q_l[8]  = config.q_l5;
      q_l[9]  = config.q_l6;
      q_l[10] = config.q_l7;

      q_r[0]  = config.q_base1;
      q_r[1]  = config.q_base2;
      q_r[2]  = config.q_base3;
      q_r[3]  = config.q_base4;
      q_r[4]  = config.q_r1;
      q_r[5]  = config.q_r2;
      q_r[6]  = config.q_r3;
      q_r[7]  = config.q_r4;
      q_r[8]  = config.q_r5;
      q_r[9]  = config.q_r6;
      q_r[10] = config.q_r7;
    }

    using Pr2ParamConfigServer = dynamic_reconfigure::Server<ahl_robot_samples::Pr2ParamConfig>;
    using Pr2ParamConfigServerPtr = std::shared_ptr<Pr2ParamConfigServer>;

    Pr2ParamConfigServerPtr server_;
    dynamic_reconfigure::Server<ahl_robot_samples::Pr2ParamConfig>::CallbackType f_;
  };

  using Pr2ParamPtr = std::shared_ptr<Pr2Param>;

} // namespace ahl_sample

#endif // __AHL_ROBOT_SAMPLES_PR2_PARAM_HPP
