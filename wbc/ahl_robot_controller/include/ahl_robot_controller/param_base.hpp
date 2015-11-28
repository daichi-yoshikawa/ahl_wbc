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

#ifndef __AHL_ROBOT_CONTROLLER_PARAM_BASE_HPP
#define __AHL_ROBOT_CONTROLLER_PARAM_BASE_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_ctrl
{

  class ParamBase
  {
  public:
    virtual ~ParamBase() {}

    virtual const Eigen::MatrixXd& getKpJoint() = 0;
    virtual const Eigen::MatrixXd& getKvJoint() = 0;
    virtual const Eigen::MatrixXd& getKpTask() = 0;
    virtual const Eigen::MatrixXd& getKiTask() = 0;
    virtual const Eigen::MatrixXd& getKvTask() = 0;
    virtual const Eigen::MatrixXd& getKvDamp() = 0;
    virtual const Eigen::MatrixXd& getKpLimit() = 0;
    virtual const Eigen::MatrixXd& getKvLimit() = 0;
    virtual const Eigen::Vector3d& getIClippingTaskPos() = 0;
    virtual const Eigen::Vector3d& getIClippingTaskOri() = 0;
    virtual double getJointErrorMax() = 0;
    virtual double getPosErrorMax() = 0;
    virtual double getOriErrorMax() = 0;
    virtual double getDqMax() = 0;
    virtual double getVxMax() = 0;
    virtual double getKpWheel() = 0;
    virtual double getKvWheel() = 0;
    virtual const Eigen::Vector3d& getG() = 0;
    virtual const Eigen::MatrixXd& getB() = 0;
  };

  typedef boost::shared_ptr<ParamBase> ParamBasePtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_PARAM_BASE_HPP */
