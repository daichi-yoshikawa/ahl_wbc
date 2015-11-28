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

#ifndef __AHL_ROBOT_TRANSFORMATION_HPP
#define __AHL_ROBOT_TRANSFORMATION_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_robot
{

  class Transformation
  {
  public:
    Transformation();
    virtual ~Transformation() {}
    virtual const Eigen::Matrix4d& T(double q) = 0;
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T) = 0;
    virtual Eigen::Vector3d& axis()
    {
      return axis_;
    }
  protected:
    Eigen::Matrix4d T_;
    Eigen::Vector3d axis_;
  };
  typedef boost::shared_ptr<Transformation> TransformationPtr;

  class Fixed : public Transformation
  {
  public:
    virtual const Eigen::Matrix4d& T(double q)
    {
      return T_;
    }
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T)
    {
    }
  };

  class RevoluteX : public Transformation
  {
  public:
    RevoluteX();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  private:
    Eigen::Matrix3d R_;
  };

  class RevoluteY : public Transformation
  {
  public:
    RevoluteY();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  private:
    Eigen::Matrix3d R_;
  };

  class RevoluteZ : public Transformation
  {
  public:
    RevoluteZ();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  private:
    Eigen::Matrix3d R_;
  };

  class PrismaticX : public Transformation
  {
  public:
    PrismaticX();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  };

  class PrismaticY : public Transformation
  {
  public:
    PrismaticY();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  };

  class PrismaticZ : public Transformation
  {
  public:
    PrismaticZ();
    virtual const Eigen::Matrix4d& T(double q);
    virtual void transform(double q, const Eigen::Matrix4d& T_org, Eigen::Matrix4d& T);
  };

}

#endif /* __AHL_ROBOT_TRANSFORMATION_HPP */
