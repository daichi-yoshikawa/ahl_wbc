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

#include <iostream>
#include "ahl_utils/exception.hpp"
#include "ahl_robot_samples/mobility/mecanum_wheel.hpp"

using namespace ahl_sample;

MecanumWheel::MecanumWheel(const Eigen::Vector4d& q, double tread_width, double wheel_base, double wheel_radius)
{
  if(tread_width <= 0.0)
  {
    std::stringstream msg;
    msg << "Invalid tread width." << std::endl
        << "  tread width : " << tread_width << std::endl;
    throw ahl_utils::Exception("MecanumWheel::MecanumWheel", msg.str());
  }

  if(wheel_base <= 0.0)
  {
    std::stringstream msg;
    msg << "Invalid tread wheel base." << std::endl
        << "  wheel base : " << wheel_base << std::endl;
    throw ahl_utils::Exception("MecanumWheel::MecanumWheel", msg.str());
  }

  if(wheel_radius <= 0.0)
  {
    std::stringstream msg;
    msg << "Invalid wheel radius." << std::endl
        << "  wheel radius : " << wheel_radius << std::endl;
    throw ahl_utils::Exception("MecanumWheel::MecanumWheel", msg.str());
  }

  q_ = q;
  dq_ = Eigen::Vector4d::Zero();

  double l1 = 0.5 * tread_width;
  double l2 = 0.5 * wheel_base;

  decomposer_.resize(4, 3);
  decomposer_ <<
    1.0, -1.0, -(l1 + l2),
    1.0,  1.0,   l1 + l2,
    1.0,  1.0, -(l1 + l2),
    1.0, -1.0,   l1 + l2;
  decomposer_ *= (1.0 / wheel_radius);
}

void MecanumWheel::update(const Eigen::Vector3d& v_base, double period)
{
  dq_ = decomposer_ * v_base;
  q_ = q_ + dq_ * period;
  for(uint32_t i = 0; i < q_.rows(); ++i)
  {
    q_[i] = atan2(sin(q_[i]), cos(q_[i]));
  }
}
