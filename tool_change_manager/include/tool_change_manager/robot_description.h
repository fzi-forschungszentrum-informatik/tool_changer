// Copyright 2025 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*!\file tool_change_manager/robot_description.h
 * \brief Complete robot description including kinematics and semantics
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-22
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_MANAGER_ROBOT_DESCRIPTION_H_INCLUDED
#define TOOL_CHANGE_MANAGER_ROBOT_DESCRIPTION_H_INCLUDED

#include <string>

namespace rclcpp {
class Node;
}

namespace tool_change_manager {

struct RobotDescription
{
  static RobotDescription fromParams(rclcpp::Node& node, const std::string& ns = "");

  std::string robot_description;
  std::string robot_description_semantic;
};

} // namespace tool_change_manager

#endif // TOOL_CHANGE_MANAGER_ROBOT_DESCRIPTION_H_INCLUDED
