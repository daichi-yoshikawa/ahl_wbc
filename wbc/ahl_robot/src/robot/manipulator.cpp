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

#include <ros/ros.h>
#include <ahl_digital_filter/pseudo_differentiator.hpp>
#include "ahl_robot/definition.hpp"
#include "ahl_utils/exception.hpp"
#include "ahl_robot/robot/manipulator.hpp"
#include "ahl_robot/utils/math.hpp"

using namespace ahl_robot;

Manipulator::Manipulator()
  : name_(""), dof_(0), updated_joint_(false)
{
}

void Manipulator::init(unsigned int init_dof, const Eigen::VectorXd& init_q)
{
  if(init_dof != init_q.rows())
  {
    std::stringstream msg;
    msg << "dof != init_q.rows()" << std::endl
        << "  dof           : " << init_dof << std::endl
        << "  init_q.rows() : " << init_q.rows();
    throw ahl_utils::Exception("Manipulator::init", msg.str());
  }

  dof_ = init_dof;

  // Resize vectors and matrices
  q_  = Eigen::VectorXd::Zero(dof_);
  dq_ = Eigen::VectorXd::Zero(dof_);

  T_.resize(link_.size());
  for(unsigned int i = 0; i < T_.size(); ++i)
  {
    T_[i] = link_[i]->T_org;
  }

  T_abs_.resize(dof_ + 1);
  for(unsigned int i = 0; i < T_abs_.size(); ++i)
  {
    T_abs_[i] = Eigen::Matrix4d::Identity();
  }
  C_abs_.resize(dof_ + 1);
  for(unsigned int i = 0; i < C_abs_.size(); ++i)
  {
    C_abs_[i] = Eigen::Matrix4d::Identity();
  }
  Pin_.resize(dof_ + 1);
  for(unsigned int i = 0; i < Pin_.size(); ++i)
  {
    Pin_[i] = Eigen::Vector3d::Zero();
  }

  q_ = init_q;

  differentiator_ = ahl_filter::DifferentiatorPtr(
    new ahl_filter::PseudoDifferentiator(update_rate_, cutoff_frequency_));
  differentiator_->init(q_, dq_);
  this->computeForwardKinematics();

  J_.resize(link_.size());
  M_.resize(dof_, dof_);
  M_inv_.resize(dof_, dof_);

  for(unsigned int i = 0; i < link_.size(); ++i)
  {
    name_to_idx_[link_[i]->name] = i;
  }
}

void Manipulator::update(const Eigen::VectorXd& q_msr)
{
  if(q_msr.rows() != dof_)
  {
    std::stringstream msg;
    msg << "q_.rows() != dof" << std::endl
        << "  q_.rows   : " << q_msr.rows() << std::endl
        << "  dof : " << dof_;
    throw ahl_utils::Exception("Manipulator::update", msg.str());
  }

  q_ = q_msr;
  this->computeForwardKinematics();
  this->computeVelocity();
  updated_joint_ = true;
}

void Manipulator::update(const Eigen::VectorXd& q_msr, const Eigen::VectorXd& dq_msr)
{
  if(q_msr.rows() != dof_)
  {
    std::stringstream msg;
    msg << "q_.rows() != dof" << std::endl
        << "  q_.rows   : " << q_msr.rows() << std::endl
        << "  dof : " << dof_;
    throw ahl_utils::Exception("Manipulator::update", msg.str());
  }

  if(dq_msr.rows() != dof_)
  {
    std::stringstream msg;
    msg << "dq_.rows() != dof" << std::endl
        << "  dq_.rows   : " << dq_msr.rows() << std::endl
        << "  dof       : " << dof_;
    throw ahl_utils::Exception("Manipulator::update", msg.str());
  }

  q_  = q_msr;
  dq_ = dq_msr;
  this->computeForwardKinematics();
  updated_joint_ = true;
}

void Manipulator::computeJacobian()
{
  if(J_.size() != link_.size())
  {
    std::stringstream msg;
    msg << "J0.size() != link_.size()" << std::endl
        << "  J0.size   : " << J_.size() << std::endl
        << "  link_.size : " << link_.size();
    throw ahl_utils::Exception("Manipulator::computeJacobian", msg.str());
  }

  for(unsigned int i = 0; i < link_.size(); ++i)
  {
    this->computeJacobian(i, J_[i]);
  }
}

void Manipulator::computeMassMatrix()
{
  M_ = Eigen::MatrixXd::Zero(M_.rows(), M_.cols());

  for(unsigned int i = 0; i < link_.size(); ++i)
  {
    Eigen::MatrixXd Jv = J_[i].block(0, 0, 3, J_[i].cols());
    Eigen::MatrixXd Jw = J_[i].block(3, 0, 3, J_[i].cols());

    M_ += link_[i]->m * Jv.transpose() * Jv + Jw.transpose() * link_[i]->I * Jw;
  }

  if(M_.rows() > macro_dof_ && M_.cols() > macro_dof_)
  {
    // TODO : Can I really ignore this coupling !?
    Eigen::MatrixXd M_macro = Eigen::MatrixXd::Zero(macro_dof_, macro_dof_);

    for(unsigned int i = 0; i < macro_dof_; ++i)
    {
      M_macro.coeffRef(i, i) = M_.coeff(i, i);
      M_inv_.coeffRef(i, i) = 1.0 / M_.coeff(i, i);
    }
    M_.block(0, 0, macro_dof_, macro_dof_) = M_macro;

    M_.block(0, macro_dof_, macro_dof_, M_.cols() - macro_dof_) = Eigen::MatrixXd::Zero(macro_dof_, M_.cols() - macro_dof_);
    M_.block(macro_dof_, 0, M_.rows() - macro_dof_, macro_dof_) = Eigen::MatrixXd::Zero(M_.rows() - macro_dof_, macro_dof_);
  }
  else
  {
    std::stringstream msg;
    msg << "Mass matrix size is not invalid." << std::endl
        << "  macr_manipulator_dof : " << macro_dof_ << std::endl
        << "  M.rows    : " << M_.rows() << std::endl
        << "  M.cols    : " << M_.cols();
    throw ahl_utils::Exception("Manipulator::computeMassMatrix", msg.str());
  }

  unsigned int mini_dof = dof_ - macro_dof_;
  M_inv_.block(macro_dof_, macro_dof_, mini_dof, mini_dof) = M_.block(macro_dof_, macro_dof_, mini_dof, mini_dof).inverse();
}

bool Manipulator::reached(const Eigen::VectorXd& qd, double threshold)
{
  if(qd.rows() != q_.rows())
  {
    std::stringstream msg;
    msg << "qd.rows() != q_.rows()" << std::endl
        << "  qd.rows() : " << qd.rows() << std::endl
        << "  q_.rows()  : " << q_.rows();
    throw ahl_utils::Exception("Manipulator::reached", msg.str());
  }

  if((q_ - qd).norm() < threshold)
  {
    return true;
  }

  return false;
}

bool Manipulator::hasLink(const std::string& name)
{
  if(name_to_idx_.find(name) == name_to_idx_.end())
  {
    return false;
  }

  return true;
}

void Manipulator::addLink(const LinkPtr& link)
{
  link_.push_back(link);
}

void Manipulator::reverseLink()
{
  std::reverse(link_.begin(), link_.end());
}

void Manipulator::print()
{
  std::cout << "name : " << name_ << std::endl
            << "dof : " << dof_ << std::endl
            << "q_ : " << std::endl << q_ << std::endl
            << "dq_ : " << std::endl << dq_ << std::endl;

  for(unsigned int i = 0; i < T_.size(); ++i)
  {
    std::cout << "T_[" << i << "] :" << std::endl << T_[i] << std::endl;
  }

  for(unsigned int i = 0; i < link_.size(); ++i)
  {
    link_[i]->print();
  }
}

void Manipulator::computeForwardKinematics()
{
  int idx = 0;

  // Relative transformation matrix
  for(unsigned int i = 0; i < link_.size(); ++i)
  {
    if(link_[i]->joint_type == joint::FIXED)
      continue;

    link_[i]->tf->transform(q_.coeff(idx), link_[i]->T_org, T_[i]);
    ++idx;
  }

  // Absolute transformation matrix
  this->computeTabs();
  this->computeCabs();

  // Compute distance between i-th link and end-effector w.r.t base
  if(T_abs_.size() != C_abs_.size())
  {
    std::stringstream msg;
    msg << "T_abs_.size() != C_abs_.size()" << std::endl
        << "  T_abs_.size    : " << T_abs_.size() << std::endl
        << "  C_abs_.size : " << C_abs_.size();
    throw ahl_utils::Exception("Manipulator::computeForwardKinematics", msg.str());
  }
  else if(T_abs_.size() != Pin_.size())
  {
    std::stringstream msg;
    msg << "T_abs_.size() != Pin_.size()" << std::endl
        << "  T_abs_.size    : " << T_abs_.size() << std::endl
        << "  Pin_.size : " << Pin_.size();
    throw ahl_utils::Exception("Manipulator::computeForwardKinematics", msg.str());
  }
  else if(Pin_.size() == 0)
  {
    std::stringstream msg;
    msg << "Pin_.size() == 0";
    throw ahl_utils::Exception("Manipulator::computeTabs", msg.str());
  }

  Eigen::MatrixXd Pbn = Eigen::MatrixXd::Constant(4, 1, 1.0);
  Pbn.block(0, 0, 3, 1) = T_abs_[T_abs_.size() - 1].block(0, 3, 3, 1);

  for(unsigned int i = 0; i < Pin_.size(); ++i)
  {
    Eigen::Matrix4d Tib;
    math::calculateInverseTransformationMatrix(T_abs_[i], Tib);
    Eigen::MatrixXd Pin = Tib * Pbn;
    Pin_[i] = Pin.block(0, 0, 3, 1);
  }
}

void Manipulator::computeTabs()
{
  if(T_abs_.size() != T_.size())
  {
    std::stringstream msg;
    msg << "T_abs_.size() != T_.size()" << std::endl
        << "  T_abs_.size : " << T_abs_.size() << std::endl
        << "  T.size      : " << T_.size();
    throw ahl_utils::Exception("Manipulator::computeTabs", msg.str());
  }
  else if(T_abs_.size() == 0 || T_.size() == 0)
  {
    std::stringstream msg;
    msg << "T_abs_.size() == 0 || T_.size() == 0" << std::endl
        << "  T_abs_.size    : " << T_abs_.size() << std::endl
        << "  T.size : " << T_.size();
    throw ahl_utils::Exception("Manipulator::computeTabs", msg.str());
  }

  T_abs_.front() = T_.front();
  for(unsigned int i = 1; i < T_abs_.size(); ++i)
  {
    T_abs_[i] = T_abs_[i - 1] * T_[i];
  }
}

void Manipulator::computeCabs()
{
  if(C_abs_.size() != T_abs_.size())
  {
    std::stringstream msg;
    msg << "C_abs_.size() != T_abs_.size()" << std::endl
        << "  C_abs_.size : " << C_abs_.size() << std::endl
        << "  T_abs_.size : " << T_abs_.size();
    throw ahl_utils::Exception("Manipulator::computeCabs", msg.str());
  }
  else if(C_abs_.size() != link_.size())
  {
    std::stringstream msg;
    msg << "C_abs_.size() != link_.size()" << std::endl
        << "  C_abs_.size : " << C_abs_.size() << std::endl
        << "  link_.size   : " << link_.size();
    throw ahl_utils::Exception("Manipulator::computeCabs", msg.str());
  }
  else if(C_abs_.size() == 0)
  {
    std::stringstream msg;
    msg << "C_abs_.size() == 0" << std::endl
        << "  C_abs_.size    : " << C_abs_.size();
    throw ahl_utils::Exception("Manipulator::computeCabs", msg.str());
  }

  for(unsigned int i = 0; i < C_abs_.size(); ++i)
  {
    Eigen::Matrix4d Tlc = Eigen::Matrix4d::Identity();
    Tlc.block(0, 3, 3, 1) = link_[i]->C;
    C_abs_[i] = T_abs_[i] * Tlc;
  }
}

void Manipulator::computeJacobian(int idx, Eigen::MatrixXd& J)
{
  J = Eigen::MatrixXd::Zero(6, dof_);

  if(idx < dof_) // Not required to consider end-effector
  {
    for(unsigned int i = 0; i <= idx; ++i)
    {
      if(link_[i]->ep) // joint_type is prismatic
      {
        J.block(0, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link_[i]->tf->axis();
        J.block(3, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * Eigen::Vector3d::Zero();
      }
      else // joint_type is revolute
      {
        Eigen::Matrix4d Tib;
        math::calculateInverseTransformationMatrix(T_abs_[i], Tib);
        Eigen::Matrix4d Cin = Tib * C_abs_[idx];
        Eigen::Vector3d P = Cin.block(0, 3, 3, 1);

        J.block(0, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link_[i]->tf->axis().cross(P);
        J.block(3, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link_[i]->tf->axis();
      }
    }
  }
  else // Required to consider the offset of end-effector
  {
    --idx;
    for(unsigned int i = 0; i <= idx; ++i)
    {
      if(link_[i]->ep) // joint_type is prismatic
      {
        J.block(0, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link_[i]->tf->axis();
        J.block(3, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * Eigen::Vector3d::Zero();
      }
      else // joint_type is revolute
      {
        Eigen::Matrix4d Tib;
        math::calculateInverseTransformationMatrix(T_abs_[i], Tib);
        Eigen::Matrix4d Cin = Tib * C_abs_[idx];
        Eigen::Vector3d P = Cin.block(0, 3, 3, 1);

        J.block(0, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link_[i]->tf->axis().cross(P);
        J.block(3, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link_[i]->tf->axis();
      }
    }

    Eigen::MatrixXd J_Pne = Eigen::MatrixXd::Identity(6, 6);
    Eigen::Vector3d Pne;
    if(C_abs_.size() - 1 - 1 >= 0.0)
    {
      Pne = T_abs_[T_abs_.size() - 1].block(0, 3, 3, 1) - C_abs_[C_abs_.size() - 1 - 1].block(0, 3, 3, 1);
    }
    else
    {
      std::stringstream msg;
      msg << "C_abs_.size() <= 1" << std::endl
          << "Manipulator doesn't have enough links." << std::endl;
      throw ahl_utils::Exception("Manipulator::computeJacobian", msg.str());
    }

    Eigen::Matrix3d Pne_cross;
    Pne_cross <<           0.0,  Pne.coeff(2), -Pne.coeff(1),
                 -Pne.coeff(2),           0.0,  Pne.coeff(0),
                  Pne.coeff(1), -Pne.coeff(0),           0.0;
    J_Pne.block(0, 3, 3, 3) = Pne_cross;
    J = J_Pne * J;
  }
}

void Manipulator::computeVelocity()
{
  if(updated_joint_)
  {
    differentiator_->apply(q_);
    differentiator_->copyDerivativeValueTo(dq_);
  }
}
