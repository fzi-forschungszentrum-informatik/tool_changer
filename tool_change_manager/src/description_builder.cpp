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
 * \date    2025-04-14
 *
 */
//----------------------------------------------------------------------
#include "tool_change_manager/description_builder.h"

#include "tool_change_manager/srdf_builder.h"
#include "tool_change_manager/urdf_builder.h"

#include <fmt/format.h>
#include <rclcpp/logging.hpp>
#include <srdfdom/model.h>
#include <srdfdom/srdf_writer.h>
#include <tinyxml2.h>
#include <urdf_parser/urdf_parser.h>

namespace tool_change_manager {

DescriptionBuilder::DescriptionBuilder(rclcpp::Logger log)
  : m_log{std::move(log)}
{
}

RobotDescription DescriptionBuilder::buildDescription(const Component& base,
                                                      const std::vector<Tool>& tools) const
{
  const auto urdf = buildRobotDescription(base, tools);

  // Print URDF
  std::unique_ptr<tinyxml2::XMLDocument> urdf_document{urdf::exportURDF(*urdf)};
  tinyxml2::XMLPrinter printer;
  urdf_document->Accept(&printer);

  return RobotDescription{std::string{printer.CStr(), static_cast<std::size_t>(printer.CStrSize())},
                          buildRobotDescriptionSemantic(base, tools, *urdf)};
}

std::shared_ptr<urdf::ModelInterface>
DescriptionBuilder::buildRobotDescription(const Component& base,
                                          const std::vector<Tool>& tools) const
{
  UrdfBuilder builder{base.urdf_model->getName()};
  builder.copy(*base.urdf_model);

  RCLCPP_INFO(m_log, "Creating composite robot description");
  for (const auto& tool : tools)
  {
    // Find tool root link
    const auto root = tool.component.urdf_model->getRoot();
    if (!root)
    {
      RCLCPP_ERROR(m_log, "Missing root link for tool '%s'", tool.name().c_str());
      continue;
    }

    RCLCPP_INFO(m_log,
                "  - Adding tool '%s' at root '%s' to '%s'",
                tool.name().c_str(),
                root->name.c_str(),
                tool.parent.c_str());

    builder.copy(*tool.component.urdf_model);

    // Create new fixed joint to attach tool
    urdf::Joint attachment_joint;
    attachment_joint.name = fmt::format("{}_{}_{}_joint", tool.name(), root->name, tool.parent);
    attachment_joint.type = urdf::Joint::FIXED;
    attachment_joint.child_link_name  = root->name;
    attachment_joint.parent_link_name = tool.parent;
    builder.copy(attachment_joint);
  }

  return builder.build();
}

std::string DescriptionBuilder::buildRobotDescriptionSemantic(
  const Component& base, const std::vector<Tool>& tools, const urdf::ModelInterface& urdf) const
{
  rclcpp::Logger log = m_log;
  SrdfBuilder builder{base.urdf_model, base.srdf_model, log.get_child("srdf_builder")};

  // Add elements for each tool
  RCLCPP_INFO(m_log,
              "Creating composite semantic description based on '%s'",
              base.srdf_model->getName().c_str());
  for (const auto& tool : tools)
  {
    RCLCPP_INFO(m_log, " - Adding SRDF model '%s'", tool.component.srdf_model->getName().c_str());
    builder.addModel(tool.component.srdf_model);

    const auto parent_link = base.urdf_model->getLink(tool.parent);
    if (!parent_link)
    {
      throw std::runtime_error{
        fmt::format("Could not find parent link '{}' of tool '{}'", tool.parent, tool.name())};
    }

    // Create fixed collision pairs
    const auto fixed_collisions =
      createFixedCollisions(tool.component.urdf_model->getRoot(), parent_link);
    RCLCPP_INFO(m_log, "   Adding %zu disabled collisions", fixed_collisions.size());
    for (const auto& fixed_collision : fixed_collisions)
    {
      builder.addDisabledCollision(
        fixed_collision.link1_, fixed_collision.link2_, fixed_collision.reason_);
    }

    // Check if tool was attached to a chain
    if (tool.active_parent)
    {
      const auto& chain = tool.active_parent->chains_[0];

      // Automatically create chains
      for (const auto& tip_link : tool.tip_links)
      {
        const auto chain_name = fmt::format("{}_{}", tool.active_parent->name_, tip_link->name);
        RCLCPP_INFO(m_log,
                    "     Automatically creating chain '%s' from '%s' to '%s'",
                    chain_name.c_str(),
                    chain.first.c_str(),
                    tip_link->name.c_str());

        builder.addChain(chain_name, chain.first, tip_link->name);
      }

      // Automatically create end effector tags
      for (const auto& tool_group : tool.component.srdf_model->getGroups())
      {
        RCLCPP_INFO(m_log,
                    "     Automatically creating end-effector for tool group '%s'",
                    tool_group.name_.c_str());
        builder.addEndEffector(
          tool_group.name_, tool_group.name_, parent_link->name, tool.active_parent->name_);
      }
    }
  }

  // Print SRDF
  const auto srdf = builder.build(urdf);
  srdf::SRDFWriter writer;
  writer.initModel(*base.urdf_model, *srdf);
  return writer.getSRDFString();
}

std::vector<srdf::Model::CollisionPair> DescriptionBuilder::createFixedCollisions(
  const std::shared_ptr<const urdf::Link>& tool_root,
  const std::shared_ptr<const urdf::Link>& tool_parent) const
{
  // Find fixed reachable links for tool and attachment point and add disabled collision tags for
  // each combination
  const auto attachment_fixed_reachable = fixedReachableChildren(fixedReachableRoot(tool_parent));
  const auto tool_fixed_reachable       = fixedReachableChildren(tool_root);

  std::vector<srdf::Model::CollisionPair> result;
  for (const auto& base_link : attachment_fixed_reachable)
  {
    for (const auto& tool_link : tool_fixed_reachable)
    {
      RCLCPP_DEBUG(m_log, "     %s - %s", base_link->name.c_str(), tool_link->name.c_str());
      result.emplace_back(base_link->name, tool_link->name, "Never");
    }
  }

  return result;
}

std::vector<std::shared_ptr<const urdf::Link>>
DescriptionBuilder::fixedReachableChildren(const std::shared_ptr<const urdf::Link>& link) const
{
  std::vector<std::shared_ptr<const urdf::Link>> result;
  result.push_back(link);

  for (const auto& child : link->child_links)
  {
    if (!child || !child->parent_joint)
    {
      throw std::runtime_error{"Missing references to child links"};
    }

    // Skip children connected by non-fixed joints
    if (child->parent_joint->type != urdf::Joint::FIXED)
    {
      continue;
    }

    // Add recursively found children
    const auto child_reachable_children = fixedReachableChildren(child);
    result.insert(result.end(), child_reachable_children.begin(), child_reachable_children.end());
  }

  return result;
}

std::shared_ptr<const urdf::Link>
DescriptionBuilder::fixedReachableRoot(const std::shared_ptr<const urdf::Link>& link) const
{
  // Case: link is already the model root
  if (!link->parent_joint)
  {
    return link;
  }

  // Case: This is the highest link that is connected by fixed joints
  if (link->parent_joint->type != urdf::Joint::FIXED)
  {
    return link;
  }

  // Otherwise go up the tree
  const auto parent_link = link->getParent();
  if (!parent_link)
  {
    throw std::runtime_error{fmt::format(
      "Missing parent link for link '{}' joint '{}'", link->name, link->parent_joint->name)};
  }
  return fixedReachableRoot(parent_link);
}

} // namespace tool_change_manager
