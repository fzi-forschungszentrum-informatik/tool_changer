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
#include "tool_change_executor/tool_change_executor.h"

#include <Eigen/Geometry>
#include <fmt/format.h>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tool_change_executor {

ToolChangeExecutor::ToolChangeExecutor(const rclcpp::Node::SharedPtr& node)
  : m_node{node}
  , m_log{node->get_logger()}
  , m_tf_buf{node->get_clock()}
  , m_tf_listener{m_tf_buf, node}
  , m_tool_library{*node, m_log.get_child("tool_library")}
  , m_mapi_state_sub{node->create_subscription<MapiStateMsg>(
      "manipulation_pipeline/state",
      rclcpp::QoS{1}.transient_local().reliable(),
      [this](const MapiStateMsg& msg) { m_mapi_generation = msg.generation; })}
  , m_execute_path_client{rclcpp_action::create_client<ExecutePathAction>(
      node, "manipulation_pipeline/execute_path")}
  , m_set_tool_client{node->create_client<SetToolSrv>("tool_change_manager/set_tool")}
  , m_couple_thread{std::bind(&ToolChangeExecutor::couple,
                              this,
                              std::placeholders::_1,
                              std::placeholders::_2),
                    m_log.get_child("couple_work_thread")}
  , m_decouple_thread{std::bind(&ToolChangeExecutor::decouple,
                                this,
                                std::placeholders::_1,
                                std::placeholders::_2),
                      m_log.get_child("decouple_work_thread")}
  , m_couple_server{rclcpp_action::create_server<CoupleAction>(
      node,
      "~/couple",
      std::bind(
        &ToolChangeExecutor::coupleGoalCb, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ToolChangeExecutor::coupleCancelCb, this, std::placeholders::_1),
      std::bind(&ToolChangeExecutor::coupleAcceptCb, this, std::placeholders::_1))}
  , m_decouple_server{rclcpp_action::create_server<DecoupleAction>(
      node,
      "~/decouple",
      std::bind(
        &ToolChangeExecutor::decoupleGoalCb, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ToolChangeExecutor::decoupleCancelCb, this, std::placeholders::_1),
      std::bind(&ToolChangeExecutor::decoupleAcceptCb, this, std::placeholders::_1))}
{
  m_tool_library.setUpdateCb(
    std::bind(&ToolChangeExecutor::updateToolClients, this, std::placeholders::_1));
  RCLCPP_INFO(m_log, "Tool change executor started");
}

void ToolChangeExecutor::updateToolClients(const std::vector<ToolLibrary::ToolDefinitionMsg>& tools)
{
  RCLCPP_INFO(m_log, "Updating tool clients");
  m_lock_clients.clear();
  m_unlock_clients.clear();

  for (const auto& tool : tools)
  {
    if (!tool.pre_lock_action.empty() && !m_lock_clients.contains(tool.pre_lock_action))
    {
      m_lock_clients.insert(
        std::make_pair(tool.pre_lock_action,
                       rclcpp_action::create_client<LockAction>(m_node, tool.pre_lock_action)));
    }
    if (!tool.lock_action.empty() && !m_lock_clients.contains(tool.lock_action))
    {
      m_lock_clients.insert(std::make_pair(
        tool.lock_action, rclcpp_action::create_client<LockAction>(m_node, tool.lock_action)));
    }

    if (!tool.pre_unlock_action.empty() && !m_unlock_clients.contains(tool.pre_unlock_action))
    {
      m_unlock_clients.insert(
        std::make_pair(tool.pre_unlock_action,
                       rclcpp_action::create_client<UnlockAction>(m_node, tool.pre_unlock_action)));
    }
    if (!tool.unlock_action.empty() && !m_unlock_clients.contains(tool.unlock_action))
    {
      m_unlock_clients.insert(
        std::make_pair(tool.unlock_action,
                       rclcpp_action::create_client<UnlockAction>(m_node, tool.unlock_action)));
    }
  }
}

void ToolChangeExecutor::couple(const std::shared_ptr<CoupleGoalHandle>& goal_handle,
                                std::stop_token stop)
{
  const auto result = std::make_shared<CoupleAction::Result>();
  try
  {
    const auto& goal = *goal_handle->get_goal();
    RCLCPP_INFO(m_log,
                "Coupling tool %s to link %s using joint group %s",
                goal.tool_name.c_str(),
                goal.tip.c_str(),
                goal.joint_group.c_str());

    // Get tool definition from library
    const auto [tool, tool_state] = m_tool_library.lookup(goal.tool_name);
    if (tool_state.active)
    {
      throw std::runtime_error{
        fmt::format("Cannot couple tool '{}' as it is already coupled to group '{}' (link '{}')",
                    tool.name,
                    tool_state.active_parent_group,
                    tool_state.parent)};
    }

    // Check that we are able to execute
    if (stop.stop_requested())
    {
      throw std::runtime_error{"Stop requested"};
    }

    // Call pre-lock action
    callLock(tool.pre_lock_action, m_lock_clients, goal.tip, tool.name);

    // Execute first motion based on tool definition
    ExecutePathAction::Goal couple_before_lock_goal{};
    couple_before_lock_goal.joint_group   = goal.joint_group;
    couple_before_lock_goal.tip           = goal.tip;
    couple_before_lock_goal.path_frame    = tool.couple_before_lock.path_frame;
    couple_before_lock_goal.path          = tool.couple_before_lock.path;
    couple_before_lock_goal.motion_limits = tool.couple_before_lock.motion_limits;

    couple_before_lock_goal.disabled_collisions = goal.allowed_collisions;
    for (const auto& collision_link : goal.before_lock_collision_links)
    {
      for (const auto& tool_link : tool.links)
      {
        manipulation_pipeline_interfaces::msg::CollisionPair collision_pair{};
        collision_pair.link1 = collision_link;
        collision_pair.link2 = tool_link;
        couple_before_lock_goal.disabled_collisions.push_back(collision_pair);
      }
    }

    executePath(couple_before_lock_goal);

    // Call lock action
    callLock(tool.lock_action, m_lock_clients, goal.tip, tool.name);

    // Attach tool and wait for manipulation pipeline to start up again
    const auto mapi_generation_before = m_mapi_generation;
    setTool(tool.name, goal.tip);
    while (m_mapi_generation == mapi_generation_before)
    {
      RCLCPP_INFO_THROTTLE(
        m_log, *m_node->get_clock(), 1000, "Waiting for manipulation pipeline to restart");
      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    // Execute retract motion
    ExecutePathAction::Goal couple_after_lock_goal{};
    couple_after_lock_goal.joint_group         = goal.joint_group;
    couple_after_lock_goal.tip                 = tool.couple_after_lock.tip;
    couple_after_lock_goal.path_frame          = tool.couple_after_lock.path_frame;
    couple_after_lock_goal.path                = tool.couple_after_lock.path;
    couple_after_lock_goal.motion_limits       = tool.couple_after_lock.motion_limits;
    couple_after_lock_goal.disabled_collisions = goal.allowed_collisions;

    executePath(couple_after_lock_goal);

    result->success = true;
    result->message =
      fmt::format("Successfully coupled tool '{}' to link '{}'", tool.name, goal.tip);
    goal_handle->succeed(result);
  }
  catch (std::runtime_error& ex)
  {
    result->success = false;
    result->message = fmt::format("Could not couple: {}", ex.what());
    goal_handle->abort(result);
  }
}

void ToolChangeExecutor::decouple(const std::shared_ptr<DecoupleGoalHandle>& goal_handle,
                                  std::stop_token stop)
{
  const auto result = std::make_shared<DecoupleAction::Result>();
  try
  {
    const auto& goal = *goal_handle->get_goal();
    RCLCPP_INFO(m_log,
                "Decoupling tool %s at target frame %s",
                goal.tool_name.c_str(),
                goal.target_frame.c_str());

    const auto now = m_node->get_clock()->now();

    // Get tool definition from library
    const auto [tool, tool_state] = m_tool_library.lookup(goal.tool_name);
    if (!tool_state.active)
    {
      throw std::runtime_error{
        fmt::format("Cannot decouple tool '{}' as it is currently not active", tool.name)};
    }

    // Check that we are able to execute
    if (stop.stop_requested())
    {
      throw std::runtime_error{"Stop requested"};
    }

    // Call pre-unlock action
    callUnlock(tool.pre_unlock_action, m_unlock_clients, tool.name);

    // Execute first motion, adjusting path based on final path frame transform
    const auto now_tf            = tf2::TimePoint{std::chrono::nanoseconds{now.nanoseconds()}};
    const auto tip_in_tool_frame = m_tf_buf.transform(
      tf2::Stamped{Eigen::Isometry3d::Identity(), now_tf, tool.decouple_before_unlock.tip},
      tool_state.parent);

    ExecutePathAction::Goal decouple_before_unlock_goal{};
    decouple_before_unlock_goal.joint_group         = tool_state.active_parent_group;
    decouple_before_unlock_goal.tip                 = tool.decouple_before_unlock.tip;
    decouple_before_unlock_goal.path_frame          = goal.target_frame;
    decouple_before_unlock_goal.disabled_collisions = goal.allowed_collisions;
    std::transform(tool.decouple_before_unlock.path.begin(),
                   tool.decouple_before_unlock.path.end(),
                   std::back_inserter(decouple_before_unlock_goal.path),
                   [&](const auto& p) {
                     Eigen::Isometry3d pose;
                     tf2::convert(p, pose);

                     geometry_msgs::msg::Pose result_msg;
                     tf2::convert(tip_in_tool_frame * pose, result_msg);
                     return result_msg;
                   });
    decouple_before_unlock_goal.motion_limits = tool.decouple_before_unlock.motion_limits;

    executePath(decouple_before_unlock_goal);

    // Call unlock action
    callUnlock(tool.unlock_action, m_unlock_clients, tool.name);

    // Detach tool and wait for manipulation pipeline to start up again
    const auto mapi_generation_before = m_mapi_generation;
    setTool(tool.name, goal.target_frame);
    while (m_mapi_generation == mapi_generation_before)
    {
      RCLCPP_INFO_THROTTLE(
        m_log, *m_node->get_clock(), 1000, "Waiting for manipulation pipeline to restart");
      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    // Execute retract motion
    ExecutePathAction::Goal decouple_after_unlock_goal{};
    decouple_after_unlock_goal.joint_group   = tool_state.active_parent_group;
    decouple_after_unlock_goal.tip           = tool.decouple_after_unlock.tip;
    decouple_after_unlock_goal.path_frame    = tool.decouple_after_unlock.path_frame;
    decouple_after_unlock_goal.path          = tool.decouple_after_unlock.path;
    decouple_after_unlock_goal.motion_limits = tool.decouple_after_unlock.motion_limits;

    decouple_after_unlock_goal.disabled_collisions = goal.allowed_collisions;
    for (const auto& collision_link : goal.after_unlock_collision_links)
    {
      for (const auto& tool_link : tool.links)
      {
        manipulation_pipeline_interfaces::msg::CollisionPair collision_pair{};
        collision_pair.link1 = collision_link;
        collision_pair.link2 = tool_link;
        decouple_after_unlock_goal.disabled_collisions.push_back(collision_pair);
      }
    }

    executePath(decouple_after_unlock_goal);

    result->success = true;
    result->message =
      fmt::format("Successfully decoupled tool '{}' at link '{}'", tool.name, goal.target_frame);
    goal_handle->succeed(result);
  }
  catch (std::runtime_error& ex)
  {
    result->success = false;
    result->message = fmt::format("Could not decouple: {}", ex.what());
    goal_handle->abort(result);
  }
}

void ToolChangeExecutor::executePath(const ExecutePathAction::Goal& goal)
{
  const auto result = callAction(goal, *m_execute_path_client);

  if (!result->success)
  {
    throw std::runtime_error{fmt::format("Could not execute path: {}", result->message)};
  }
  RCLCPP_INFO(m_log, "Successfully executed path: %s", result->message.c_str());
}

void ToolChangeExecutor::callLock(
  const std::string& action,
  const std::unordered_map<std::string, rclcpp_action::Client<LockAction>::SharedPtr>& clients,
  const std::string& tip,
  const std::string& tool_name) const
{
  if (action.empty())
  {
    RCLCPP_INFO(m_log, "No lock action defined");
  }
  else if (const auto it = clients.find(action); it == clients.end())
  {
    throw std::runtime_error{fmt::format("No client for action '{}'", action)};
  }
  else
  {
    LockAction::Goal goal;
    goal.tip          = tip;
    goal.tool_name    = tool_name;
    const auto result = callAction(goal, *it->second);

    if (!result->success)
    {
      throw std::runtime_error{fmt::format("Could not call lock action: {}", result->message)};
    }
    RCLCPP_INFO(m_log,
                "Successfully called locking action '%s': %s",
                action.c_str(),
                result->message.c_str());
  }
}

void ToolChangeExecutor::callUnlock(
  const std::string& action,
  const std::unordered_map<std::string, rclcpp_action::Client<UnlockAction>::SharedPtr>& clients,
  const std::string& tool_name) const
{
  if (action.empty())
  {
    RCLCPP_INFO(m_log, "No unlock action defined");
  }
  else if (const auto it = clients.find(action); it == clients.end())
  {
    throw std::runtime_error{fmt::format("No client for action '{}'", action)};
  }
  else
  {
    UnlockAction::Goal goal;
    goal.tool_name    = tool_name;
    const auto result = callAction(goal, *it->second);

    if (!result->success)
    {
      throw std::runtime_error{fmt::format("Could not call unlock action: {}", result->message)};
    }
    RCLCPP_INFO(m_log,
                "Successfully called unlocking action '%s': %s",
                action.c_str(),
                result->message.c_str());
  }
}

void ToolChangeExecutor::setTool(const std::string& tool_name, const std::string& parent_link)
{
  if (!m_set_tool_client->service_is_ready())
  {
    throw std::runtime_error{"SetTool service is not yet ready"};
  }

  const auto request   = std::make_shared<SetToolSrv::Request>();
  request->tool_name   = tool_name;
  request->parent_link = parent_link;

  auto request_future = m_set_tool_client->async_send_request(request);
  const auto result   = request_future.get();

  if (!result->success)
  {
    throw std::runtime_error{fmt::format(
      "Could not attach tool '{}' to link '{}': {}", tool_name, parent_link, result->message)};
  }

  RCLCPP_INFO(m_log,
              "Successfully attached tool '%s' to link '%s': %s",
              tool_name.c_str(),
              parent_link.c_str(),
              result->message.c_str());
}

rclcpp_action::GoalResponse
ToolChangeExecutor::coupleGoalCb(const rclcpp_action::GoalUUID& uuid,
                                 std::shared_ptr<const CoupleAction::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ToolChangeExecutor::coupleAcceptCb(std::shared_ptr<CoupleGoalHandle> goal_handle)
{
  if (!m_couple_thread.work(goal_handle))
  {
    const auto result = std::make_shared<CoupleAction::Result>();
    result->success   = true;
    result->message   = "Couple action already in progress";
    goal_handle->abort(result);
  }
}

rclcpp_action::CancelResponse
ToolChangeExecutor::coupleCancelCb(std::shared_ptr<CoupleGoalHandle> goal_handle)
{
  return rclcpp_action::CancelResponse::REJECT;
}

rclcpp_action::GoalResponse
ToolChangeExecutor::decoupleGoalCb(const rclcpp_action::GoalUUID& uuid,
                                   std::shared_ptr<const DecoupleAction::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void ToolChangeExecutor::decoupleAcceptCb(std::shared_ptr<DecoupleGoalHandle> goal_handle)
{
  if (!m_decouple_thread.work(goal_handle))
  {
    const auto result = std::make_shared<DecoupleAction::Result>();
    result->success   = true;
    result->message   = "Decouple action already in progress";
    goal_handle->abort(result);
  }
}

rclcpp_action::CancelResponse
ToolChangeExecutor::decoupleCancelCb(std::shared_ptr<DecoupleGoalHandle> goal_handle)
{
  return rclcpp_action::CancelResponse::REJECT;
}

} // namespace tool_change_executor


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>("tool_change_executor");
  const tool_change_executor::ToolChangeExecutor tool_change_executor{node};
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
