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

/*!\file tool_change_executor/tool_library.h
 * \brief Local library of all defined tools and their state
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-05-06
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_EXECUTOR_TOOL_LIBRARY_H_INCLUDED
#define TOOL_CHANGE_EXECUTOR_TOOL_LIBRARY_H_INCLUDED

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <tool_change_interfaces/msg/state.hpp>
#include <tool_change_interfaces/msg/tool_definition.hpp>
#include <tool_change_interfaces/msg/tool_library.hpp>
#include <tool_change_interfaces/msg/tool_state.hpp>
#include <utility>

namespace tool_change_executor {

class ToolLibrary
{
public:
  using ToolDefinitionMsg = tool_change_interfaces::msg::ToolDefinition;
  using ToolStateMsg      = tool_change_interfaces::msg::ToolState;

  ToolLibrary(rclcpp::Node& node, rclcpp::Logger log);

  using UpdateCb = std::function<void(const std::vector<ToolDefinitionMsg>&)>;
  void setUpdateCb(UpdateCb cb);

  [[nodiscard]] std::pair<ToolDefinitionMsg, ToolStateMsg> lookup(const std::string& name) const;

private:
  using ToolLibraryMsg = tool_change_interfaces::msg::ToolLibrary;
  using StateMsg       = tool_change_interfaces::msg::State;

  rclcpp::Logger m_log;

  rclcpp::Subscription<ToolLibraryMsg>::SharedPtr m_tool_library_sub;
  rclcpp::Subscription<StateMsg>::SharedPtr m_tool_state_sub;

  UpdateCb m_update_cb;

  ToolLibraryMsg::ConstSharedPtr m_last_library_msg;
  StateMsg::ConstSharedPtr m_last_state_msg;
  mutable std::mutex m_tool_mut;
};

} // namespace tool_change_executor

#endif // TOOL_CHANGE_EXECUTOR_TOOL_LIBRARY_H_INCLUDED
