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

/*!\file tool_change_manager/tool_change_manager.h
 * \brief Central tool change manager node
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-03
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_MANAGER_TOOL_CHANGE_MANAGER_H_INCLUDED
#define TOOL_CHANGE_MANAGER_TOOL_CHANGE_MANAGER_H_INCLUDED

#include "param_interface.h"
#include "tool.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tool_change_interfaces/msg/state.hpp>
#include <tool_change_interfaces/msg/tool_library.hpp>
#include <tool_change_interfaces/msg/tool_path.hpp>
#include <tool_change_interfaces/srv/set_tool.hpp>
#include <vector>

namespace tool_change_manager {

class RobotModel;

class ToolChangeManager
{
public:
  explicit ToolChangeManager(const rclcpp::Node::SharedPtr& node);

private:
  using ToolPathMsg = tool_change_interfaces::msg::ToolPath;

  std::vector<ToolDefinition> readToolParams(rclcpp::Node& node) const;
  ToolPathMsg readToolPath(const std::string& prefix,
                           const std::string& default_tip,
                           const std::string& default_frame,
                           rclcpp::Node& node) const;

  void publishToollibrary(const std::vector<Tool>& tools) const;
  void toolChangeCb(const std::vector<Tool>& tools) const;

  using SetTool = tool_change_interfaces::srv::SetTool;
  void setToolCb(std::shared_ptr<rmw_request_id_t> header,
                 std::shared_ptr<SetTool::Request> request);

  rclcpp::Node::SharedPtr m_node;
  rclcpp::Logger m_log;

  ParamInterface m_param_interface;
  std::shared_ptr<RobotModel> m_robot_model;

  rclcpp::Service<SetTool>::SharedPtr m_set_tool_service;

  using StateMsg = tool_change_interfaces::msg::State;
  rclcpp::Publisher<StateMsg>::SharedPtr m_state_pub;

  using LibraryMsg = tool_change_interfaces::msg::ToolLibrary;
  rclcpp::Publisher<LibraryMsg>::SharedPtr m_library_pub;
};

} // namespace tool_change_manager

#endif // TOOL_CHANGE_MANAGER_TOOL_CHANGE_MANAGER_H_INCLUDED
