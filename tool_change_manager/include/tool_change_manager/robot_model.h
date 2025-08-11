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

/*!\file tool_change_manager/robot_model.h
 * \brief Model of robot structure including tools
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-23
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_MANAGER_ROBOT_MODEL_H_INCLUDED
#define TOOL_CHANGE_MANAGER_ROBOT_MODEL_H_INCLUDED

#include "description_builder.h"
#include "robot_description.h"
#include "tool.h"

#include <functional>
#include <memory>
#include <rclcpp/logger.hpp>
#include <string>
#include <urdf_model/pose.h>
#include <vector>

namespace urdf {
class ModelInterface;
class Joint;
class Link;
} // namespace urdf
namespace srdf {
class Model;
}

namespace tool_change_manager {

class UrdfBuilder;
class SrdfBuilder;

class RobotModel
{
public:
  RobotModel(const RobotDescription& robot_description,
             const std::vector<ToolDefinition>& tools,
             rclcpp::Logger log);

  using ChangeCb = std::function<void(const std::vector<Tool>&)>;
  void setChangeCb(ChangeCb cb);

  [[nodiscard]] const std::vector<Tool>& tools() const;

  void attach(const std::string& tool_name, const std::string& link);

  [[nodiscard]] RobotDescription createDescription() const;

private:
  void updateActive(Tool& tool) const;

  void parseInitialModel(const RobotDescription& robot_description,
                         const std::vector<ToolDefinition>& tools);
  void searchToolsRecursive(const urdf::Link& cur_link,
                            const std::vector<ToolDefinition>& tools,
                            const Component& base_model,
                            UrdfBuilder& base_builder);

  void initializeTool(const urdf::Link& root_link,
                      const ToolDefinition& tool,
                      const Component& base_model);
  void searchToolRecursive(const urdf::Link& cur_link, UrdfBuilder& urdf_builder);

  rclcpp::Logger m_log;

  Component m_base_model;
  std::vector<Tool> m_tools;

  ChangeCb m_change_cb;

  DescriptionBuilder m_description_builder;
};

} // namespace tool_change_manager

#endif // TOOL_CHANGE_MANAGER_ROBOT_MODEL_H_INCLUDED
