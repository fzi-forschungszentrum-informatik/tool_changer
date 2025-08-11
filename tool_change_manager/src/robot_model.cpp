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
 * \date    2025-04-23
 *
 */
//----------------------------------------------------------------------
#include "tool_change_manager/robot_model.h"

#include "tool_change_manager/srdf_builder.h"
#include "tool_change_manager/urdf_builder.h"

#include <algorithm>
#include <fmt/format.h>
#include <rclcpp/logging.hpp>
#include <srdfdom/model.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace tool_change_manager {

RobotModel::RobotModel(const RobotDescription& robot_description,
                       const std::vector<ToolDefinition>& tools,
                       rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_base_model{std::make_shared<urdf::ModelInterface>(), std::make_shared<srdf::Model>()}
  , m_description_builder{m_log.get_child("builder")}
{
  parseInitialModel(robot_description, tools);
}

void RobotModel::setChangeCb(ChangeCb cb)
{
  m_change_cb = std::move(cb);
}

const std::vector<Tool>& RobotModel::tools() const
{
  return m_tools;
}

void RobotModel::attach(const std::string& tool_name, const std::string& link)
{
  const auto it = std::find_if(
    m_tools.begin(), m_tools.end(), [&](const auto& tool) { return tool.name() == tool_name; });
  if (it == m_tools.end())
  {
    throw std::runtime_error{fmt::format("Tool '{}' does not exist", tool_name)};
  }
  auto& tool = *it;

  // Verify that link actually exists
  const auto urdf_link = m_base_model.urdf_model->getLink(link);
  if (!urdf_link)
  {
    throw std::runtime_error{fmt::format("Parent link '{}' does not exist", link)};
  }

  // Update linking information
  const bool changed = tool.parent != link;
  tool.parent        = link;
  tool.pose.clear();

  // Update active state
  updateActive(tool);

  // Call change callback
  if (changed && m_change_cb)
  {
    m_change_cb(m_tools);
  }
}

RobotDescription RobotModel::createDescription() const
{
  return m_description_builder.buildDescription(m_base_model, m_tools);
}

void RobotModel::updateActive(Tool& tool) const
{
  tool.active_parent = nullptr;
  for (const auto& group : m_base_model.srdf_model->getGroups())
  {
    // Check single-chain groups
    if (group.joints_.empty() && group.links_.empty() && group.subgroups_.empty() &&
        (group.chains_.size() == 1))
    {
      const auto& chain = group.chains_[0];
      if (tool.parent == chain.second)
      {
        // Check that this is unique
        if (tool.active_parent)
        {
          throw std::runtime_error{fmt::format(
            "Attaching tool '{}' to link '{}' does not result in a unique active parent",
            tool.name(),
            tool.parent)};
        }

        RCLCPP_DEBUG(m_log,
                     "Tool '%s' is active with parent group '%s'",
                     tool.name().c_str(),
                     group.name_.c_str());

        tool.active_parent = &group;
      }
    }
  }
}

void RobotModel::parseInitialModel(const RobotDescription& robot_description,
                                   const std::vector<ToolDefinition>& tools)
{
  // Parsing initial URDF and SRDF model
  Component base_model{urdf::parseURDF(robot_description.robot_description),
                       std::make_shared<srdf::Model>()};
  if (!base_model.urdf_model)
  {
    throw std::runtime_error{"Could not parse initial URDF model"};
  }

  if (!base_model.srdf_model->initString(*base_model.urdf_model,
                                         robot_description.robot_description_semantic))
  {
    throw std::runtime_error{"Could not parse initial SRDF model"};
  }

  SrdfBuilder srdf_builder{
    base_model.urdf_model, base_model.srdf_model, m_log.get_child("srdf_builder")};

  // Check root link
  const auto root_link = base_model.urdf_model->getRoot();
  if (!root_link)
  {
    throw std::runtime_error{"No root in initial URDF model"};
  }

  // Copy over model-wide information to new base model
  RCLCPP_DEBUG(m_log, "Setting base model name to '%s'", base_model.urdf_model->name_.c_str());
  UrdfBuilder base_builder{base_model.urdf_model->getName()};

  for (const auto& [name, material] : base_model.urdf_model->materials_)
  {
    RCLCPP_DEBUG(m_log, "Copying material '%s' to base model", name.c_str());
    base_builder.copy(*material);
  }

  // DFS to find tools and copy base model
  RCLCPP_DEBUG(m_log, "Recursively searching links in model");
  searchToolsRecursive(*root_link, tools, base_model, base_builder);

  // Create base model
  m_base_model.urdf_model = base_builder.build();

  srdf_builder.setFilter(m_base_model.urdf_model);
  m_base_model.srdf_model = srdf_builder.build(*m_base_model.urdf_model);

  // Update active state for all tools
  for (auto& tool : m_tools)
  {
    updateActive(tool);
  }
}

void RobotModel::searchToolsRecursive(const urdf::Link& cur_link,
                                      const std::vector<ToolDefinition>& tools,
                                      const Component& base_model,
                                      UrdfBuilder& base_builder)
{
  RCLCPP_DEBUG(m_log, "Recursively searching link %s", cur_link.name.c_str());

  if (const auto tool_it =
        std::find_if(tools.begin(),
                     tools.end(),
                     [&](const auto& tool) { return tool.root_link == cur_link.name; });
      tool_it != tools.end())
  {
    initializeTool(cur_link, *tool_it, base_model);
  }
  else
  {
    // Copy over link and parent joint to base model
    base_builder.copy(cur_link);
    if (cur_link.parent_joint)
    {
      base_builder.copy(*cur_link.parent_joint);
    }

    // Recursively visit children
    for (const auto& child : cur_link.child_links)
    {
      if (!child)
      {
        throw std::runtime_error{fmt::format("Child of link '{}' is null", cur_link.name)};
      }

      searchToolsRecursive(*child, tools, base_model, base_builder);
    }
  }
}

void RobotModel::initializeTool(const urdf::Link& root_link,
                                const ToolDefinition& tool,
                                const Component& base_model)
{
  RCLCPP_DEBUG(m_log, "Searching parts of tool '%s'", tool.name.c_str());

  UrdfBuilder urdf_builder{tool.name};
  SrdfBuilder srdf_builder{
    base_model.urdf_model, base_model.srdf_model, m_log.get_child("srdf_builder")};

  searchToolRecursive(root_link, urdf_builder);

  // Check out parent to get current attachment status
  if (!root_link.parent_joint)
  {
    throw std::runtime_error{fmt::format("Parent link of tool '{}' with root link '{}' is null",
                                         tool.name.c_str(),
                                         root_link.name.c_str())};
  }
  if (root_link.parent_joint->type != urdf::Joint::FIXED)
  {
    throw std::runtime_error{
      fmt::format("Tool '{}' is currently attached by joint '{}' with type {}, but only fixed "
                  "joints are supported",
                  tool.name,
                  root_link.parent_joint->name,
                  root_link.parent_joint->type)};
  }

  // Create tool
  Tool result;
  result.component.urdf_model = urdf_builder.build();

  srdf_builder.setFilter(result.component.urdf_model);
  result.component.srdf_model = srdf_builder.build(*result.component.urdf_model);

  result.parent = root_link.parent_joint->parent_link_name;
  result.pose   = root_link.parent_joint->parent_to_joint_origin_transform;

  // Use tool URDF to resolve tip links
  for (const auto& tip_link_name : tool.tip_links)
  {
    const auto link = result.component.urdf_model->getLink(tip_link_name);
    if (!link)
    {
      throw std::runtime_error{
        fmt::format("Missing tip link '{}' for tool '{}'", tip_link_name, tool.name.c_str())};
    }
    result.tip_links.push_back(link.get());
  }

  const auto tip_links_str = fmt::format("[{}]", fmt::join(tool.tip_links, ", "));
  RCLCPP_DEBUG(m_log,
               "Adding tool '%s', currently attached to '%s' with pose (%.02f, %.02f, "
               "%.02f)[%.02f, %.02f, %.02f, %.02f] (tip links: %s)",
               tool.name.c_str(),
               result.parent.c_str(),
               result.pose.position.x,
               result.pose.position.y,
               result.pose.position.z,
               result.pose.rotation.x,
               result.pose.rotation.y,
               result.pose.rotation.z,
               result.pose.rotation.w,
               tip_links_str.c_str());
  m_tools.push_back(result);
}

void RobotModel::searchToolRecursive(const urdf::Link& cur_link, UrdfBuilder& urdf_builder)
{
  RCLCPP_DEBUG(
    m_log, "Searching link '%s' for tool '%s'", cur_link.name.c_str(), urdf_builder.name().c_str());

  // Copy over child joints
  urdf_builder.copy(cur_link);
  for (const auto& joint : cur_link.child_joints)
  {
    if (!joint)
    {
      throw std::runtime_error{fmt::format("Child joint of link '{}' is null", joint->name)};
    }

    urdf_builder.copy(*joint);
  }

  for (const auto& child : cur_link.child_links)
  {
    if (!child)
    {
      throw std::runtime_error{fmt::format("Child of link '{}' is null", child->name)};
    }

    searchToolRecursive(*child, urdf_builder);
  }
}

} // namespace tool_change_manager
