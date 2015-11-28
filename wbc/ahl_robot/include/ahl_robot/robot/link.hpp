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

#ifndef __AHL_ROBOT_LINK_HPP
#define __AHL_ROBOT_LINK_HPP

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot/robot/transformation.hpp"

namespace ahl_robot
{
  class Link;
  typedef boost::shared_ptr<Link> LinkPtr;

  class Link
  {
  public:
    Link ()
      : name(""), joint_type(""), parent(""), child(""), ep(false),
        m(0.0), q_min(0.0), q_max(0.0), dq_max(0.0), tau(0.0), tau_max(0.0)
    {
      T_org = Eigen::Matrix4d::Identity();
      C = Eigen::Vector3d::Zero();
      I = Eigen::Matrix3d::Zero();
    }

    void print()
    {
      std::cout << "name : " << name << std::endl
                << "joint_type : " << joint_type << std::endl
                << "parent : " << parent << std::endl
                << "child : " << child << std::endl
                << "ep    : " << ep << std::endl
                << "T_org : " << std::endl << T_org << std::endl
                << "C : " << std::endl << C << std::endl
                << "m : " << m << std::endl
                << "I : " << std::endl << I << std::endl
                << "q_min : " << q_min << std::endl
                << "q_max : " << q_max << std::endl
                << "dq_max : " << dq_max << std::endl
                << "tau : " << tau << std::endl
                << "tau_max : " << tau_max << std::endl;
    }

    std::string name;
    std::string joint_type;
    std::string parent;
    std::string child;
    bool ep;

    TransformationPtr tf;

    Eigen::Matrix4d T_org;
    Eigen::Vector3d C;

    double m;
    Eigen::Matrix3d I;

    double q_min;
    double q_max;
    double dq_max;
    double tau;
    double tau_max;
  };

}

#endif /* __AHL_ROBOT_LINK_HPP */
