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

/*!\file tool_change_executor/tool_change_executor.h
 * \brief Central node for tool change execution
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-05-06
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_EXECUTOR_TOOL_CHANGE_EXECUTOR_H_INCLUDED
#define TOOL_CHANGE_EXECUTOR_TOOL_CHANGE_EXECUTOR_H_INCLUDED

#include "tool_library.h"
#include "work_thread.h"

#include <geometry_msgs/msg/pose.hpp>
#include <manipulation_pipeline_interfaces/action/execute_path.hpp>
#include <manipulation_pipeline_interfaces/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tool_change_interfaces/action/couple.hpp>
#include <tool_change_interfaces/action/decouple.hpp>
#include <tool_change_interfaces/action/lock.hpp>
#include <tool_change_interfaces/action/unlock.hpp>
#include <tool_change_interfaces/srv/set_tool.hpp>
#include <unordered_map>

namespace tool_change_executor {

class ToolChangeExecutor
{
public:
  explicit ToolChangeExecutor(const rclcpp::Node::SharedPtr& node);

private:
  template <typename ActionT>
  typename ActionT::Result::SharedPtr callAction(const typename ActionT::Goal& goal,
                                                 rclcpp_action::Client<ActionT>& client) const;

  using CoupleAction     = tool_change_interfaces::action::Couple;
  using CoupleGoalHandle = rclcpp_action::ServerGoalHandle<CoupleAction>;

  using DecoupleAction     = tool_change_interfaces::action::Decouple;
  using DecoupleGoalHandle = rclcpp_action::ServerGoalHandle<DecoupleAction>;

  using LockAction   = tool_change_interfaces::action::Lock;
  using UnlockAction = tool_change_interfaces::action::Unlock;

  using ExecutePathAction = manipulation_pipeline_interfaces::action::ExecutePath;
  using SetToolSrv        = tool_change_interfaces::srv::SetTool;
  using MapiStateMsg      = manipulation_pipeline_interfaces::msg::State;

  void updateToolClients(const std::vector<ToolLibrary::ToolDefinitionMsg>& tools);

  void couple(const std::shared_ptr<CoupleGoalHandle>& goal_handle, std::stop_token stop);
  void decouple(const std::shared_ptr<DecoupleGoalHandle>& goal_handle, std::stop_token stop);

  void executePath(const ExecutePathAction::Goal& goal);
  void callLock(
    const std::string& action,
    const std::unordered_map<std::string, rclcpp_action::Client<LockAction>::SharedPtr>& clients,
    const std::string& tip,
    const std::string& tool_name) const;
  void callUnlock(
    const std::string& action,
    const std::unordered_map<std::string, rclcpp_action::Client<UnlockAction>::SharedPtr>& clients,
    const std::string& tool_name) const;
  void setTool(const std::string& tool_name, const std::string& parent_link);

  rclcpp_action::GoalResponse coupleGoalCb(const rclcpp_action::GoalUUID& uuid,
                                           std::shared_ptr<const CoupleAction::Goal> goal);
  void coupleAcceptCb(std::shared_ptr<CoupleGoalHandle> goal_handle);
  rclcpp_action::CancelResponse coupleCancelCb(std::shared_ptr<CoupleGoalHandle> goal_handle);

  rclcpp_action::GoalResponse decoupleGoalCb(const rclcpp_action::GoalUUID& uuid,
                                             std::shared_ptr<const DecoupleAction::Goal> goal);
  void decoupleAcceptCb(std::shared_ptr<DecoupleGoalHandle> goal_handle);
  rclcpp_action::CancelResponse decoupleCancelCb(std::shared_ptr<DecoupleGoalHandle> goal_handle);

  rclcpp::Node::SharedPtr m_node;
  rclcpp::Logger m_log;

  tf2_ros::Buffer m_tf_buf;
  tf2_ros::TransformListener m_tf_listener;

  ToolLibrary m_tool_library;
  std::size_t m_mapi_generation;

  std::unordered_map<std::string, rclcpp_action::Client<LockAction>::SharedPtr> m_lock_clients;
  std::unordered_map<std::string, rclcpp_action::Client<UnlockAction>::SharedPtr> m_unlock_clients;

  rclcpp::Subscription<MapiStateMsg>::SharedPtr m_mapi_state_sub;
  rclcpp_action::Client<ExecutePathAction>::SharedPtr m_execute_path_client;
  rclcpp::Client<SetToolSrv>::SharedPtr m_set_tool_client;

  WorkThread<std::shared_ptr<CoupleGoalHandle>> m_couple_thread;
  WorkThread<std::shared_ptr<DecoupleGoalHandle>> m_decouple_thread;

  rclcpp_action::Server<CoupleAction>::SharedPtr m_couple_server;
  rclcpp_action::Server<DecoupleAction>::SharedPtr m_decouple_server;
};

} // namespace tool_change_executor


namespace tool_change_executor {

template <typename ActionT>
typename ActionT::Result::SharedPtr
ToolChangeExecutor::callAction(const typename ActionT::Goal& goal,
                               rclcpp_action::Client<ActionT>& client) const
{
  if (!client.action_server_is_ready())
  {
    throw std::runtime_error{"Action server not ready"};
  }

  const auto goal_handle = client.async_send_goal(goal).get();
  if (!goal_handle)
  {
    throw std::runtime_error{"Goal rejected by server"};
  }

  const auto result = client.async_get_result(goal_handle).get();
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;

    case rclcpp_action::ResultCode::ABORTED:
      throw std::runtime_error{"Goal aborted"};
    case rclcpp_action::ResultCode::CANCELED:
      throw std::runtime_error{"Goal canceled"};
    default:
      throw std::runtime_error{"Unknown goal result code"};
  }

  return result.result;
}

} // namespace tool_change_executor

#endif // TOOL_CHANGE_EXECUTOR_TOOL_CHANGE_EXECUTOR_H_INCLUDED
