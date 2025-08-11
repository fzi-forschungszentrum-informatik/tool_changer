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

/*!\file
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-05-06
 *
 */
//----------------------------------------------------------------------
#include "tool_change_executor/tool_library.h"

#include <fmt/format.h>

namespace tool_change_executor {

ToolLibrary::ToolLibrary(rclcpp::Node& node, rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_tool_library_sub{node.create_subscription<ToolLibraryMsg>(
      "tool_change_manager/tool_library",
      rclcpp::QoS{1}.transient_local().reliable(),
      [this](const ToolLibraryMsg::ConstSharedPtr& msg) {
        RCLCPP_INFO(m_log, "Received tool library consisting of %zu tools", msg->tools.size());
        {
          std::lock_guard lock{m_tool_mut};
          m_last_library_msg = msg;

          if (m_update_cb)
          {
            m_update_cb(m_last_library_msg->tools);
          }
        }
      })}
  , m_tool_state_sub{
      node.create_subscription<StateMsg>("tool_change_manager/state",
                                         rclcpp::QoS{1}.transient_local().reliable(),
                                         [this](const StateMsg::ConstSharedPtr& msg) {
                                           std::lock_guard lock{m_tool_mut};
                                           m_last_state_msg = msg;
                                         })}
{
  RCLCPP_INFO(m_log, "Waiting for initial tool library...");
}

void ToolLibrary::setUpdateCb(UpdateCb cb)
{
  m_update_cb = cb;
}

std::pair<ToolLibrary::ToolDefinitionMsg, ToolLibrary::ToolStateMsg>
ToolLibrary::lookup(const std::string& name) const
{
  std::lock_guard lock{m_tool_mut};
  if (!m_last_library_msg)
  {
    throw std::runtime_error{"No tool library received yet"};
  }
  if (!m_last_state_msg)
  {
    throw std::runtime_error{"No tool state received yet"};
  }

  const auto library_find_it = std::find_if(m_last_library_msg->tools.begin(),
                                            m_last_library_msg->tools.end(),
                                            [&](const auto& tool) { return tool.name == name; });
  if (library_find_it == m_last_library_msg->tools.end())
  {
    throw std::runtime_error{fmt::format("Could not find tool '{}' in tool library", name)};
  }

  const auto state_find_it = std::find_if(m_last_state_msg->tool_states.begin(),
                                          m_last_state_msg->tool_states.end(),
                                          [&](const auto& tool) { return tool.name == name; });
  if (state_find_it == m_last_state_msg->tool_states.end())
  {
    throw std::runtime_error{fmt::format("Did not receive tool state for tool '{}' yet", name)};
  }

  return std::make_pair(*library_find_it, *state_find_it);
}

} // namespace tool_change_executor
