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

#include "ahl_robot/robot/transformation.hpp"

using namespace ahl_robot;

Transformation::Transformation()
{
  T_ = Eigen::Matrix4d::Identity();
}

RevoluteX::RevoluteX()
{
  R_ = Eigen::Matrix3d::Identity();
  axis_ << 1, 0, 0;
}

const Eigen::Matrix4d& RevoluteX::T(double q)
{
  R_ = Eigen::AngleAxisd(q, axis_);
  T_.block(0, 0, 3, 3) = R_;
  return T_;
}

void RevoluteX::transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
{
  T = T_org;
  T.block(0, 0, 3, 3) = T_org.block(0, 0, 3, 3) * this->T(q).block(0, 0, 3, 3);
}

RevoluteY::RevoluteY()
{
  R_ = Eigen::Matrix3d::Identity();
  axis_ << 0, 1, 0;
}

const Eigen::Matrix4d& RevoluteY::T(double q)
{
  R_ = Eigen::AngleAxisd(q, axis_);
  T_.block(0, 0, 3, 3) = R_;
  return T_;
}

void RevoluteY::transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
{
  T = T_org;
  T.block(0, 0, 3, 3) = T_org.block(0, 0, 3, 3) * this->T(q).block(0, 0, 3, 3);
}

RevoluteZ::RevoluteZ()
{
  R_ = Eigen::Matrix3d::Identity();
  axis_ << 0, 0, 1;
}

const Eigen::Matrix4d& RevoluteZ::T(double q)
{
  R_ = Eigen::AngleAxisd(q, axis_);
  T_.block(0, 0, 3, 3) = R_;
  return T_;
}

void RevoluteZ::transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
{
  T = T_org;
  T.block(0, 0, 3, 3) = T_org.block(0, 0, 3, 3) * this->T(q).block(0, 0, 3, 3);
}

PrismaticX::PrismaticX()
{
  axis_ << 1, 0, 0;
}

const Eigen::Matrix4d& PrismaticX::T(double q)
{
  T_.coeffRef(0, 3) = q;
  return T_;
}

void PrismaticX::transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
{
  T = T_org;
  T.block(0, 3, 3, 1) += this->T(q).block(0, 3, 3, 1);
}

PrismaticY::PrismaticY()
{
  axis_ << 0, 1, 0;
}

const Eigen::Matrix4d& PrismaticY::T(double q)
{
  T_.coeffRef(1, 3) = q;
  return T_;
}

void PrismaticY::transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
{
  T = T_org;
  T.block(0, 3, 3, 1) += this->T(q).block(0, 3, 3, 1);
}

PrismaticZ::PrismaticZ()
{
  axis_ << 0, 0, 1;
}

const Eigen::Matrix4d& PrismaticZ::T(double q)
{
  T_.coeffRef(2, 3) = q;
  return T_;
}

void PrismaticZ::transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
{
  T = T_org;
  T.block(0, 3, 3, 1) += this->T(q).block(0, 3, 3, 1);
}
