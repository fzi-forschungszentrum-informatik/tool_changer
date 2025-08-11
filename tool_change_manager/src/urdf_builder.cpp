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
#include "tool_change_manager/urdf_builder.h"

#include <urdf_model/model.h>

namespace tool_change_manager {

UrdfBuilder::UrdfBuilder(const std::string& name)
  : m_urdf{std::make_shared<urdf::ModelInterface>()}
{
  m_urdf->name_ = name;
}

const std::string& UrdfBuilder::name() const
{
  return m_urdf->getName();
}

void UrdfBuilder::copy(const urdf::ModelInterface& model)
{
  for (const auto& [_, link] : model.links_)
  {
    copy(*link);
  }
  for (const auto& [_, joint] : model.joints_)
  {
    copy(*joint);
  }
  for (const auto& [_, material] : model.materials_)
  {
    copy(*material);
  }
}

void UrdfBuilder::copy(const urdf::Link& link)
{
  // Don't copy pointers to other links and joints, they will get added using the init functions of
  // urdf::ModelInterface. This ensures that we have actual deep copies and are able to later on add
  // joints etc without changing the base model.
  const auto result = std::make_shared<urdf::Link>();

  result->name            = link.name;
  result->inertial        = link.inertial;
  result->visual          = link.visual;
  result->collision       = link.collision;
  result->collision_array = link.collision_array;
  result->visual_array    = link.visual_array;

  m_urdf->links_.insert({link.name, result});
}

void UrdfBuilder::copy(const urdf::Joint& joint)
{
  // Copying joints directly is no problem - All shared members are not changed by tool changing
  m_urdf->joints_.insert({joint.name, std::make_shared<urdf::Joint>(joint)});
}

void UrdfBuilder::copy(const urdf::Material& material)
{
  m_urdf->materials_.insert({material.name, std::make_shared<urdf::Material>(material)});
}

std::shared_ptr<urdf::ModelInterface> UrdfBuilder::build()
{
  std::map<std::string, std::string> parent_link_tree;
  m_urdf->initTree(parent_link_tree);
  m_urdf->initRoot(parent_link_tree);

  const auto result = m_urdf;
  m_urdf            = std::make_shared<urdf::ModelInterface>();
  return result;
}

} // namespace tool_change_manager
