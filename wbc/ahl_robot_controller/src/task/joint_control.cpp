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

#include "ahl_utils/exception.hpp"
#include "ahl_robot_controller/task/joint_control.hpp"

using namespace ahl_ctrl;

JointControl::JointControl(const ahl_robot::ManipulatorPtr& mnp)
{
  mnp_ = mnp;
  N_ = Eigen::MatrixXd::Identity(mnp_->getDOF(), mnp_->getDOF());
}

void JointControl::setGoal(const Eigen::MatrixXd& qd)
{
  if(qd.rows() != mnp_->getDOF())
  {
    std::stringstream msg;
    msg << "qd.rows() != mnp_->getDOF()" << std::endl
        << "  qd.rows   : " << qd.rows() << std::endl
        << "  mnp_->dof : " << mnp_->getDOF();
    throw ahl_utils::Exception("JointControl::setGoal", msg.str());
  }

  qd_ = qd.block(0, 0, qd.rows(), 1);
}

void JointControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(mnp_->getDOF());

  Eigen::VectorXd error = qd_ - mnp_->q();
  Eigen::MatrixXd Kpv = param_->getKpJoint().block(0, 0, mnp_->getDOF(), mnp_->getDOF());

  for(unsigned int i = 0; i < Kpv.rows(); ++i)
  {
    Kpv.coeffRef(i, i) /= param_->getKvJoint().block(0, 0, mnp_->getDOF(), mnp_->getDOF()).coeff(i, i);
  }
  Eigen::VectorXd dqd = Kpv * error;

  //const double dq_max = 25.0;

  for(unsigned int i = 0; i < dqd.rows(); ++i)
  {
    if(dqd[i] < -param_->getDqMax())
    {
      dqd[i] = -param_->getDqMax();
    }
    else if(dqd[i] > param_->getDqMax())
    {
      dqd[i] = param_->getDqMax();
    }
  }

  Eigen::VectorXd tau_unit = -param_->getKvJoint().block(0, 0, mnp_->getDOF(), mnp_->getDOF()) * (mnp_->dq() - dqd);

  tau = tau_ = mnp_->getMassMatrix() * tau_unit;
}
