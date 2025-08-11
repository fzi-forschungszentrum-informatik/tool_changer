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

/*!\file tool_change_manager/description_builder.h
 * \brief Build an aggregate robot description based on the current tool state
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-14
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_MANAGER_DESCRIPTION_BUILDER_H_INCLUDED
#define TOOL_CHANGE_MANAGER_DESCRIPTION_BUILDER_H_INCLUDED

#include "robot_description.h"
#include "tool.h"

#include <rclcpp/logger.hpp>
#include <srdfdom/model.h>
#include <string>

namespace urdf {
class ModelInterface;
class Link;
} // namespace urdf

namespace tool_change_manager {

class DescriptionBuilder
{
public:
  explicit DescriptionBuilder(rclcpp::Logger log);

  [[nodiscard]] RobotDescription buildDescription(const Component& base,
                                                  const std::vector<Tool>& tools) const;

private:
  std::shared_ptr<urdf::ModelInterface> buildRobotDescription(const Component& base,
                                                              const std::vector<Tool>& tools) const;
  std::string buildRobotDescriptionSemantic(const Component& base,
                                            const std::vector<Tool>& tools,
                                            const urdf::ModelInterface& urdf) const;

  // Collision pair creation
  std::vector<srdf::Model::CollisionPair>
  createFixedCollisions(const std::shared_ptr<const urdf::Link>& tool_root,
                        const std::shared_ptr<const urdf::Link>& tool_parent) const;
  std::vector<std::shared_ptr<const urdf::Link>>
  fixedReachableChildren(const std::shared_ptr<const urdf::Link>& link) const;
  std::shared_ptr<const urdf::Link>
  fixedReachableRoot(const std::shared_ptr<const urdf::Link>& link) const;

  rclcpp::Logger m_log;
};

} // namespace tool_change_manager

#endif // TOOL_CHANGE_MANAGER_DESCRIPTION_BUILDER_H_INCLUDED
