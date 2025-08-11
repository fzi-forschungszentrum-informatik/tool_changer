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
#include "tool_change_manager/srdf_builder.h"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <srdfdom/srdf_writer.h>

namespace tool_change_manager {

SrdfBuilder::SrdfBuilder(std::shared_ptr<urdf::ModelInterface> urdf,
                         std::shared_ptr<srdf::Model> srdf,
                         rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_urdf{std::move(urdf)}
  , m_srdf{std::move(srdf)}
{
}

void SrdfBuilder::addModel(std::shared_ptr<srdf::Model> srdf)
{
  m_add_models.push_back(std::move(srdf));
}

void SrdfBuilder::addDisabledCollision(const std::string& link1,
                                       const std::string& link2,
                                       const std::string& reason)
{
  m_add_collision_pairs.emplace_back(link1, link2, reason);
}

void SrdfBuilder::addChain(const std::string& name,
                           const std::string& base_link,
                           const std::string& tip_link)
{
  srdf::Model::Group group;
  group.name_ = name;
  group.chains_.emplace_back(std::make_pair(base_link, tip_link));
  m_add_groups.push_back(group);
}

void SrdfBuilder::addEndEffector(const std::string& name,
                                 const std::string& group,
                                 const std::string& parent_link,
                                 const std::string& parent_group)
{
  m_add_end_effectors.emplace_back(name, parent_link, parent_group, group);
}

void SrdfBuilder::setFilter(std::shared_ptr<urdf::ModelInterface> urdf)
{
  m_filter = std::move(urdf);
}

std::shared_ptr<srdf::Model> SrdfBuilder::build(const urdf::ModelInterface& base_urdf)
{
  // Create writer to get access to variables
  srdf::SRDFWriter writer;
  writer.initModel(*m_urdf, *m_srdf);

  // Add additional models
  for (const auto& add_model : m_add_models)
  {
    copyModelComponents(add_model->getGroups(), writer.groups_);
    copyModelComponents(add_model->getGroupStates(), writer.group_states_);
    copyModelComponents(add_model->getVirtualJoints(), writer.virtual_joints_);
    copyModelComponents(add_model->getEndEffectors(), writer.end_effectors_);
    copyModelComponents(add_model->getLinkSphereApproximations(),
                        writer.link_sphere_approximations_);
    copyModelComponents(add_model->getNoDefaultCollisionLinks(),
                        writer.no_default_collision_links_);
    copyModelComponents(add_model->getDisabledCollisionPairs(), writer.disabled_collision_pairs_);
    copyModelComponents(add_model->getEnabledCollisionPairs(), writer.enabled_collision_pairs_);
    copyModelComponents(add_model->getPassiveJoints(), writer.passive_joints_);

    for (const auto& [joint, properties] : add_model->getJointProperties())
    {
      writer.joint_properties_.insert({joint, properties});
    }
  }

  // Add additional disabled collisions
  RCLCPP_DEBUG(m_log, "Adding %zu collision pairs", m_add_collision_pairs.size());
  copyModelComponents(m_add_collision_pairs, writer.disabled_collision_pairs_);

  // Add additional groups
  RCLCPP_DEBUG(m_log, "Adding %zu groups", m_add_groups.size());
  copyModelComponents(m_add_groups, writer.groups_);

  // Add end-effectors
  RCLCPP_DEBUG(m_log, "Adding %zu end effectors", m_add_end_effectors.size());
  copyModelComponents(m_add_end_effectors, writer.end_effectors_);

  if (m_filter)
  {
    RCLCPP_DEBUG(m_log,
                 "Filtering SRDF '%s' based on URDF '%s'",
                 writer.robot_name_.c_str(),
                 m_filter->getName().c_str());

    // Remove groups
    filterGroups(writer.groups_, *m_filter);

    // Remove group states that don't belong to group
    filterGroupStates(writer.group_states_, writer.groups_);

    // Remove collision pairs
    filterCollisionPairs(writer.disabled_collision_pairs_, *m_filter);

    // Remove end effectors based on URDF and groups
    filterEndEffectors(writer.end_effectors_, writer.groups_, *m_filter);

    writer.robot_name_ = m_filter->getName();
  }

  // Create new srdf model from result
  const auto result = std::make_shared<srdf::Model>();
  if (!result->initString(base_urdf, writer.getSRDFString()))
  {
    throw std::runtime_error{"Could not build SRDF model"};
  }
  return result;
}

void SrdfBuilder::filterGroups(std::vector<srdf::Model::Group>& groups,
                               const urdf::ModelInterface& filter) const
{
  // Filter groups
  const auto group_remove_it = std::remove_if(groups.begin(), groups.end(), [&](const auto& group) {
    const auto group_exists = groupInUrdf(group, filter);
    if (!group_exists)
    {
      RCLCPP_DEBUG(
        m_log, "Removing group '%s' for model '%s'", group.name_.c_str(), filter.getName().c_str());
    }
    return !group_exists;
  });
  groups.erase(group_remove_it, groups.end());

  // Remove groups made of subgroups afterwards
  // Gradually shrink range [groups.begin(), subgroup_remove_it] until no more changes are made.
  auto subgroup_remove_it      = groups.end();
  auto last_subgroup_remove_it = subgroup_remove_it;
  do
  {
    last_subgroup_remove_it = subgroup_remove_it;

    // Remove elements that
    subgroup_remove_it =
      std::remove_if(groups.begin(), subgroup_remove_it, [&](const srdf::Model::Group& group) {
        // Contain a subgroup that
        return std::any_of(
          group.subgroups_.begin(), group.subgroups_.end(), [&](const std::string& subgroup_name) {
            // Does not exist in [groups.begin(), subgroup_remove_it]
            const auto subgroup_exists = std::find_if(groups.begin(),
                                                      last_subgroup_remove_it,
                                                      [&](const srdf::Model::Group& other_group) {
                                                        return subgroup_name == other_group.name_;
                                                      }) != subgroup_remove_it;
            if (!subgroup_exists)
            {
              RCLCPP_DEBUG(
                m_log,
                "Removing group '%s' for model '%s' because it contains removed subgroup '%s'",
                group.name_.c_str(),
                filter.getName().c_str(),
                subgroup_name.c_str());
            }

            return !subgroup_exists;
          });
      });
  } while (subgroup_remove_it != last_subgroup_remove_it);
  groups.erase(subgroup_remove_it, groups.end());
}

bool SrdfBuilder::groupInUrdf(const srdf::Model::Group& group,
                              const urdf::ModelInterface& urdf) const
{
  if (!std::all_of(group.joints_.begin(), group.joints_.end(), [&](const auto& joint) {
        return urdf.getJoint(joint);
      }))
  {
    return false;
  }

  if (!std::all_of(group.links_.begin(), group.links_.end(), [&](const auto& link) {
        return urdf.getLink(link);
      }))
  {
    return false;
  }

  if (!std::all_of(group.chains_.begin(), group.chains_.end(), [&](const auto& chain) {
        // TODO: Don't just check chain ends
        return urdf.getLink(chain.first) && urdf.getLink(chain.second);
      }))
  {
    return false;
  }

  return true;
}

void SrdfBuilder::filterGroupStates(std::vector<srdf::Model::GroupState>& group_states,
                                    const std::vector<srdf::Model::Group>& groups) const
{
  const auto group_state_remove_it =
    std::remove_if(group_states.begin(), group_states.end(), [&](const auto& group_state) {
      const auto group_exists = std::find_if(groups.begin(), groups.end(), [&](const auto& group) {
                                  return group.name_ == group_state.group_;
                                }) != groups.end();
      if (!group_exists)
      {
        RCLCPP_DEBUG(m_log,
                     "Removing group state '%s' as it belongs to removed group '%s'",
                     group_state.name_.c_str(),
                     group_state.group_.c_str());
      }
      return !group_exists;
    });

  group_states.erase(group_state_remove_it, group_states.end());
}

void SrdfBuilder::filterCollisionPairs(std::vector<srdf::Model::CollisionPair>& collision_pairs,
                                       const urdf::ModelInterface& urdf) const
{
  const auto num_pairs_before = collision_pairs.size();

  const auto remove_it =
    std::remove_if(collision_pairs.begin(), collision_pairs.end(), [&](const auto& pair) {
      const auto links_exist = urdf.getLink(pair.link1_) && urdf.getLink(pair.link2_);
      return !links_exist;
    });

  collision_pairs.erase(remove_it, collision_pairs.end());
  const auto num_pairs_after = collision_pairs.size();

  RCLCPP_DEBUG(m_log,
               "Removed %zu of %zu collision pairs",
               (num_pairs_before - num_pairs_after),
               num_pairs_before);
}

void SrdfBuilder::filterEndEffectors(std::vector<srdf::Model::EndEffector>& end_effectors,
                                     const std::vector<srdf::Model::Group>& groups,
                                     const urdf::ModelInterface& urdf) const
{
  const auto num_ee_before = end_effectors.size();

  const auto remove_it =
    std::remove_if(end_effectors.begin(), end_effectors.end(), [&](const auto& end_effector) {
      if (const auto group_it = std::find_if(
            groups.begin(),
            groups.end(),
            [&](const auto& group) { return group.name_ == end_effector.component_group_; });
          group_it == groups.end())
      {
        RCLCPP_DEBUG(m_log,
                     "Removing end effector '%s' because its component group '%s' no longer exists",
                     end_effector.name_.c_str(),
                     end_effector.component_group_.c_str());
        return true;
      }

      if (!end_effector.parent_group_.empty())
      {
        if (const auto group_it = std::find_if(
              groups.begin(),
              groups.end(),
              [&](const auto& group) { return group.name_ == end_effector.parent_group_; });
            group_it == groups.end())
        {
          RCLCPP_DEBUG(m_log,
                       "Removing end effector '%s' because its parent group '%s' no longer exists",
                       end_effector.name_.c_str(),
                       end_effector.parent_group_.c_str());
          return true;
        }
      }

      return false;
    });

  end_effectors.erase(remove_it, end_effectors.end());
  const auto num_ee_after = end_effectors.size();

  RCLCPP_DEBUG(
    m_log, "Removed %zu of %zu end effectors", (num_ee_before - num_ee_after), num_ee_before);
}

} // namespace tool_change_manager
