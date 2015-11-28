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

#ifndef __AHL_ROBOT_CONTROLLER_PARAM_HPP
#define __AHL_ROBOT_CONTROLLER_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include "ahl_robot_controller/param_base.hpp"
#include <ahl_robot/robot/robot.hpp>
#include "ahl_robot_controller/ParamConfig.h"

namespace ahl_ctrl
{

  class Param : public ParamBase
  {
  public:
    Param(const ahl_robot::RobotPtr& robot);

    virtual const Eigen::MatrixXd& getKpJoint()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kp_joint_;
    }

    virtual const Eigen::MatrixXd& getKvJoint()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kv_joint_;
    }

    virtual const Eigen::MatrixXd& getKpTask()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kp_task_;
    }

    virtual const Eigen::MatrixXd& getKiTask()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Ki_task_;
    }

    virtual const Eigen::MatrixXd& getKvTask()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kv_task_;
    }

    virtual const Eigen::Vector3d& getIClippingTaskPos()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return i_clipping_task_pos_;
    }

    virtual const Eigen::Vector3d& getIClippingTaskOri()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return i_clipping_task_ori_;
    }

    virtual const Eigen::MatrixXd& getKvDamp()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kv_damp_;
    }

    virtual const Eigen::MatrixXd& getKpLimit()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kp_limit_;
    }

    virtual const Eigen::MatrixXd& getKvLimit()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kv_limit_;
    }

    virtual double getJointErrorMax()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return joint_error_max_;
    }

    virtual double getPosErrorMax()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return pos_error_max_;
    }

    virtual double getOriErrorMax()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return ori_error_max_;
    }

    virtual double getDqMax()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return dq_max_;
    }

    virtual double getVxMax()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return vx_max_;
    }

    virtual const Eigen::Vector3d& getG()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return g_;
    }

    virtual const Eigen::MatrixXd& getB()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return b_;
    }

    virtual double getKpWheel()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return kp_wheel_;
    }

    virtual double getKvWheel()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return kv_wheel_;
    }

  private:
    void update(ahl_robot_controller::ParamConfig& config, uint32_t level);
    typedef dynamic_reconfigure::Server<ahl_robot_controller::ParamConfig> ParamConfigServer;
    typedef boost::shared_ptr<ParamConfigServer> ParamConfigServerPtr;

    boost::mutex mutex_;

    ParamConfigServerPtr server_;
    dynamic_reconfigure::Server<ahl_robot_controller::ParamConfig>::CallbackType f_;

    unsigned int dof_;
    unsigned int macro_dof_;

    Eigen::MatrixXd Kp_joint_;
    Eigen::MatrixXd Kv_joint_;
    Eigen::MatrixXd Kp_task_;
    Eigen::MatrixXd Ki_task_;
    Eigen::MatrixXd Kv_task_;
    Eigen::Vector3d i_clipping_task_pos_;
    Eigen::Vector3d i_clipping_task_ori_;
    Eigen::MatrixXd Kv_damp_;
    Eigen::MatrixXd Kp_limit_;
    Eigen::MatrixXd Kv_limit_;
    double joint_error_max_;
    double pos_error_max_;
    double ori_error_max_;
    double dq_max_;
    double vx_max_;
    double kp_wheel_;
    double kv_wheel_;
    Eigen::Vector3d g_;
    Eigen::MatrixXd b_;
  };

  typedef boost::shared_ptr<Param> ParamPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_PARAM_HPP */
