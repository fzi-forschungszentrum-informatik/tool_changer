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
 * \date    2025-04-03
 *
 */
//----------------------------------------------------------------------
#include "tool_change_manager/tool_change_manager.h"

#include "tool_change_manager/robot_model.h"

#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace tool_change_manager {

ToolChangeManager::ToolChangeManager(const rclcpp::Node::SharedPtr& node)
  : m_node{node}
  , m_log{m_node->get_logger()}
  , m_param_interface{node, node->get_logger().get_child("param_interface")}
{
  // Get initial robot description from parameters
  const auto robot_description_future = m_param_interface.loadRobotDescription();
  if (rclcpp::spin_until_future_complete(m_node, robot_description_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error{"Could not read robot description in time"};
  }
  const auto initial_robot_description = robot_description_future.get();

  // Create robot model
  m_robot_model = std::make_shared<RobotModel>(initial_robot_description,
                                               readToolParams(*m_node),
                                               node->get_logger().get_child("robot_model"));
  m_robot_model->setChangeCb(
    std::bind(&ToolChangeManager::toolChangeCb, this, std::placeholders::_1));

  // Set description based on robot model
  const auto set_base_description_future =
    m_param_interface.setRobotDescription(m_robot_model->createDescription());
  if (rclcpp::spin_until_future_complete(m_node, set_base_description_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    throw std::runtime_error{"Could not set initial description"};
  }

  // Advertise ros interfaces
  m_set_tool_service = node->create_service<SetTool>(
    "~/set_tool",
    std::bind(&ToolChangeManager::setToolCb, this, std::placeholders::_1, std::placeholders::_2));
  m_state_pub =
    node->create_publisher<StateMsg>("~/state", rclcpp::QoS{1}.transient_local().reliable());
  m_library_pub = node->create_publisher<LibraryMsg>("~/tool_library",
                                                     rclcpp::QoS{1}.transient_local().reliable());

  // Publish initial state
  toolChangeCb(m_robot_model->tools());
  publishToollibrary(m_robot_model->tools());

  RCLCPP_INFO(m_node->get_logger(), "Tool change manager started");
}

std::vector<ToolDefinition> ToolChangeManager::readToolParams(rclcpp::Node& node) const
{
  node.declare_parameter<std::vector<std::string>>("tools");
  const auto names = node.get_parameter_or<std::vector<std::string>>("tools", {});
  if (names.empty())
  {
    throw std::runtime_error{"Missing tool names"};
  }

  std::vector<ToolDefinition> tools;
  for (const auto& name : names)
  {
    // root link
    const auto root_link_param_str = fmt::format("{}.root_link", name);
    node.declare_parameter<std::string>(root_link_param_str);
    const auto root_link = node.get_parameter_or<std::string>(root_link_param_str, "");
    if (root_link.empty())
    {
      throw std::runtime_error{fmt::format("Missing root_link parameter for tool '{}'", name)};
    }

    // tip links
    const auto tip_links_param_str = fmt::format("{}.tip_links", name);
    node.declare_parameter<std::vector<std::string>>(tip_links_param_str);
    const auto tip_links = node.get_parameter_or<std::vector<std::string>>(tip_links_param_str, {});

    tools.emplace_back(name, root_link, tip_links);
  }

  return tools;
}

ToolChangeManager::ToolPathMsg ToolChangeManager::readToolPath(const std::string& prefix,
                                                               const std::string& default_tip,
                                                               const std::string& default_frame,
                                                               rclcpp::Node& node) const
{
  const auto motions_param = fmt::format("{}.motions", prefix);
  node.declare_parameter<std::vector<std::string>>(motions_param);
  const auto motions = node.get_parameter_or<std::vector<std::string>>(motions_param, {});

  const auto tip_link_param = fmt::format("{}.tip_link", prefix);
  node.declare_parameter<std::string>(tip_link_param, default_tip);
  const auto path_frame_param = fmt::format("{}.path_frame", prefix);
  node.declare_parameter<std::string>(path_frame_param, default_frame);

  ToolPathMsg msg;
  msg.tip        = node.get_parameter_or(tip_link_param, default_tip);
  msg.path_frame = node.get_parameter_or(path_frame_param, default_frame);
  msg.motion_limits.max_trans_vel =
    node.declare_parameter<double>(fmt::format("{}.motion_limits.max_trans_vel", prefix), 0.0);
  msg.motion_limits.max_trans_acc =
    node.declare_parameter<double>(fmt::format("{}.motion_limits.max_trans_acc", prefix), 0.0);
  msg.motion_limits.max_trans_dec =
    node.declare_parameter<double>(fmt::format("{}.motion_limits.max_trans_dec", prefix), 0.0);
  msg.motion_limits.max_rot_vel =
    node.declare_parameter<double>(fmt::format("{}.motion_limits.max_rot_vel", prefix), 0.0);

  for (const auto& motion_name : motions)
  {
    const auto motion_param = fmt::format("{}.{}", prefix, motion_name);
    node.declare_parameter<std::vector<double>>(motion_param);
    const auto pose =
      node.get_parameter_or<std::vector<double>>(motion_param, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (pose.size() != 6)
    {
      throw std::runtime_error{
        fmt::format("Invalid pose specification for motion {}.{}", prefix, motion_name)};
    }

    geometry_msgs::msg::Pose waypoint;
    waypoint.position.x = pose[0];
    waypoint.position.y = pose[1];
    waypoint.position.z = pose[2];

    tf2::Quaternion waypoint_quat;
    waypoint_quat.setRPY(pose[3], pose[4], pose[5]);
    waypoint.orientation.x = waypoint_quat.x();
    waypoint.orientation.y = waypoint_quat.y();
    waypoint.orientation.z = waypoint_quat.z();
    waypoint.orientation.w = waypoint_quat.w();

    msg.path.push_back(waypoint);
  }

  return msg;
}

void ToolChangeManager::publishToollibrary(const std::vector<Tool>& tools) const
{
  LibraryMsg msg;
  for (const auto& tool : tools)
  {
    // Read tool path parameters
    const auto pre_lock_action =
      m_node->declare_parameter(fmt::format("{}.couple.pre_lock_action", tool.name()), "");
    const auto couple_before_lock = readToolPath(
      fmt::format("{}.couple.before_lock", tool.name()), "", tool.root().name, *m_node);
    const auto lock_action =
      m_node->declare_parameter<std::string>(fmt::format("{}.couple.lock_action", tool.name()), "");
    const auto couple_after_lock = readToolPath(fmt::format("{}.couple.after_lock", tool.name()),
                                                tool.root().name,
                                                tool.root().name,
                                                *m_node);

    const auto pre_unlock_action =
      m_node->declare_parameter(fmt::format("{}.decouple.pre_unlock_action", tool.name()), "");
    const auto decouple_before_unlock =
      readToolPath(fmt::format("{}.decouple.before_unlock", tool.name()),
                   tool.root().name,
                   tool.root().name,
                   *m_node);
    const auto unlock_action =
      m_node->declare_parameter(fmt::format("{}.decouple.unlock_action", tool.name()), "");
    const auto decouple_after_unlock = readToolPath(
      fmt::format("{}.decouple.after_unlock", tool.name()), "", tool.root().name, *m_node);

    tool_change_interfaces::msg::ToolDefinition tool_msg;
    tool_msg.name      = tool.name();
    tool_msg.root_link = tool.root().name;

    std::vector<urdf::LinkSharedPtr> tool_links;
    tool.component.urdf_model->getLinks(tool_links);
    std::transform(tool_links.begin(),
                   tool_links.end(),
                   std::back_inserter(tool_msg.links),
                   [](const auto& link) { return link->name; });

    for (const auto& [name, _] : tool.component.urdf_model->joints_)
    {
      tool_msg.joints.push_back(name);
    }

    tool_msg.pre_lock_action        = pre_lock_action;
    tool_msg.couple_before_lock     = couple_before_lock;
    tool_msg.lock_action            = lock_action;
    tool_msg.couple_after_lock      = couple_after_lock;
    tool_msg.pre_unlock_action      = pre_unlock_action;
    tool_msg.decouple_before_unlock = decouple_before_unlock;
    tool_msg.unlock_action          = unlock_action;
    tool_msg.decouple_after_unlock  = decouple_after_unlock;

    msg.tools.push_back(tool_msg);
  }

  m_library_pub->publish(msg);
}

void ToolChangeManager::toolChangeCb(const std::vector<Tool>& tools) const
{
  StateMsg msg;
  for (const auto& tool : tools)
  {
    tool_change_interfaces::msg::ToolState tool_state;

    tool_state.name   = tool.name();
    tool_state.parent = tool.parent;

    // Set active state
    if (tool.active_parent)
    {
      tool_state.active              = true;
      tool_state.active_parent_group = tool.active_parent->name_;
    }
    else
    {
      tool_state.active = false;
    }

    msg.tool_states.push_back(tool_state);
  }

  m_state_pub->publish(msg);
}

void ToolChangeManager::setToolCb(std::shared_ptr<rmw_request_id_t> header,
                                  std::shared_ptr<SetTool::Request> request)
{
  const auto response = std::make_shared<SetTool::Response>();
  RobotDescription composite_description;

  try
  {
    response->message =
      fmt::format("Attaching tool '{}' to link '{}'", request->tool_name, request->parent_link);
    m_robot_model->attach(request->tool_name, request->parent_link);

    // Create composite description
    composite_description = m_robot_model->createDescription();
    response->success     = true;
  }
  catch (std::runtime_error& ex)
  {
    response->success = false;
    response->message = fmt::format("Error: {}", ex.what());
  }

  if (response->success)
  {
    // Asynchronously set parameter and send response
    std::thread async_executor{[this, header, response, composite_description] {
      try
      {
        // Try setting description parameter
        m_param_interface.setRobotDescription(composite_description).get();
        RCLCPP_INFO(m_log, "%s", response->message.c_str());
      }
      catch (std::runtime_error& ex)
      {
        // Error handling in case of e.g. invalid description
        response->success = false;
        response->message = fmt::format("Could not set robot description: {}", ex.what());
        RCLCPP_ERROR(m_log, "%s", response->message.c_str());
      }

      m_set_tool_service->send_response(*header, *response);
    }};
    async_executor.detach();
  }
  else
  {
    // Directly set response in case of failure
    m_set_tool_service->send_response(*header, *response);
    RCLCPP_ERROR(m_log, "%s", response->message.c_str());
  }
}

} // namespace tool_change_manager


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  const auto node                = std::make_shared<rclcpp::Node>("tool_change_manager");
  const auto tool_change_manager = std::make_shared<tool_change_manager::ToolChangeManager>(node);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
