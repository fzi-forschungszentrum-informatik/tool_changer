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

/*!\file tool_change_manager/srdf_builder.h
 * \brief Composite SRDF description builder
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-23
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_MANAGER_SRDF_BUILDER_H_INCLUDED
#define TOOL_CHANGE_MANAGER_SRDF_BUILDER_H_INCLUDED

#include <memory>
#include <rclcpp/logger.hpp>
#include <srdfdom/model.h>

namespace urdf {
class ModelInterface;
}

namespace tool_change_manager {

class SrdfBuilder
{
public:
  SrdfBuilder(std::shared_ptr<urdf::ModelInterface> urdf,
              std::shared_ptr<srdf::Model> srdf,
              rclcpp::Logger log);

  void addModel(std::shared_ptr<srdf::Model> srdf);
  void addDisabledCollision(const std::string& link1,
                            const std::string& link2,
                            const std::string& reason = "");
  void addChain(const std::string& name, const std::string& base_link, const std::string& tip_link);
  void addEndEffector(const std::string& name,
                      const std::string& group,
                      const std::string& parent_link,
                      const std::string& parent_group);

  void setFilter(std::shared_ptr<urdf::ModelInterface> urdf);

  std::shared_ptr<srdf::Model> build(const urdf::ModelInterface& base_urdf);

private:
  template <typename T>
  void copyModelComponents(const std::vector<T>& source, std::vector<T>& target) const;

  void filterGroups(std::vector<srdf::Model::Group>& groups,
                    const urdf::ModelInterface& urdf) const;
  void filterGroupStates(std::vector<srdf::Model::GroupState>& group_states,
                         const std::vector<srdf::Model::Group>& groups) const;
  void filterCollisionPairs(std::vector<srdf::Model::CollisionPair>& collision_pairs,
                            const urdf::ModelInterface& urdf) const;
  void filterEndEffectors(std::vector<srdf::Model::EndEffector>& end_effectors,
                          const std::vector<srdf::Model::Group>& groups,
                          const urdf::ModelInterface& urdf) const;

  bool groupInUrdf(const srdf::Model::Group& group, const urdf::ModelInterface& urdf) const;

  rclcpp::Logger m_log;

  std::shared_ptr<urdf::ModelInterface> m_urdf;
  std::shared_ptr<srdf::Model> m_srdf;

  std::vector<std::shared_ptr<srdf::Model>> m_add_models;
  std::vector<srdf::Model::CollisionPair> m_add_collision_pairs;
  std::vector<srdf::Model::Group> m_add_groups;
  std::vector<srdf::Model::EndEffector> m_add_end_effectors;

  std::shared_ptr<urdf::ModelInterface> m_filter;
};

} // namespace tool_change_manager


namespace tool_change_manager {

template <typename T>
void SrdfBuilder::copyModelComponents(const std::vector<T>& source, std::vector<T>& target) const
{
  target.insert(target.end(), source.begin(), source.end());
}

} // namespace tool_change_manager

#endif // TOOL_CHANGE_MANAGER_SRDF_BUILDER_H_INCLUDED
