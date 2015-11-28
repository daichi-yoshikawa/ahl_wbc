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

/////////////////////////////////////////////////
/// \file definition.hpp
/// \brief Declare static const values
/// \author Daichi Yoshikawa
/////////////////////////////////////////////////

#ifndef __AHL_ROBOT_DEFINITION_HPP
#define __AHL_ROBOT_DEFINITION_HPP

#include <string>

namespace ahl_robot
{
  namespace joint
  {
    //! Tag for revolute joint w.r.t x axis
    static const std::string REVOLUTE_X  = "revolute_x";
    //! Tag for revolute joint w.r.t y axis
    static const std::string REVOLUTE_Y  = "revolute_y";
    //! Tag for revolute joint w.r.t z axis
    static const std::string REVOLUTE_Z  = "revolute_z";
    //! Tag for prismatic joint w.r.t x axis
    static const std::string PRISMATIC_X = "prismatic_x";
    //! Tag for prismatic joint w.r.t y axis
    static const std::string PRISMATIC_Y = "prismatic_y";
    //! Tag for prismatic joint w.r.t z axis
    static const std::string PRISMATIC_Z = "prismatic_z";
    //! Tag for fixed joint
    static const std::string FIXED       = "fixed";
  }

  namespace frame
  {
    //! Name of global frame
    static const std::string WORLD = "map";
  }

  namespace mobility
  {
    /// \enum
    /// Mobility type
    enum Type
    {
      SENTRY_LOWER,
      FIXED,
      MOBILITY_2D,
      MOBILITY_3D,
      SENTRY_UPPER,
    };

    namespace type
    {
      //! Tag for mecanum wheel
      static const std::string MECANUM_WHEEL = "mecanum_wheel";
    }
  }
}

#endif /* __AHL_ROBOT_DEFINITION_HPP */
