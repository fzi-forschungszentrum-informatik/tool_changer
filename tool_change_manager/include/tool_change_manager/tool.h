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

/*!\file tool_change_manager/tool.h
 * \brief Complete description of a defined tool
 *
 * \author  Robert Wilbrandt <wilbrandt@fzi.de>
 * \date    2025-04-23
 *
 */
//----------------------------------------------------------------------
#ifndef TOOL_CHANGE_MANAGER_TOOL_H_INCLUDED
#define TOOL_CHANGE_MANAGER_TOOL_H_INCLUDED

#include <memory>
#include <optional>
#include <srdfdom/model.h>
#include <string>
#include <urdf_model/pose.h>
#include <vector>

namespace urdf {
class ModelInterface;
class Link;
} // namespace urdf
namespace tool_change_manager {

struct Component
{
  std::shared_ptr<urdf::ModelInterface> urdf_model;
  std::shared_ptr<srdf::Model> srdf_model;
};

struct ToolDefinition
{
  std::string name;
  std::string root_link;

  std::vector<std::string> tip_links;
};

struct Tool
{
  // Name and root link are defined in component.urdf_model
  Component component;

  std::string parent;
  urdf::Pose pose;

  const srdf::Model::Group* active_parent;

  std::vector<const urdf::Link*> tip_links;

  [[nodiscard]] const std::string& name() const;
  [[nodiscard]] const urdf::Link& root() const;
};

} // namespace tool_change_manager

#endif // TOOL_CHANGE_MANAGER_TOOL_H_INCLUDED
