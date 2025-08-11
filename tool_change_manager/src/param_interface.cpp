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
#include "tool_change_manager/param_interface.h"

#include <fmt/format.h>

namespace tool_change_manager {

ParamInterface::ParamInterface(const rclcpp::Node::SharedPtr& node, rclcpp::Logger log)
  : m_log{std::move(log)}
  , m_param_cbg{node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)}
  , m_urdf_client{std::make_shared<rclcpp::AsyncParametersClient>(
      node, "robot_state_publisher", rclcpp::ParametersQoS{}, m_param_cbg)}
  , m_srdf_client{std::make_shared<rclcpp::AsyncParametersClient>(
      node, "srdf_publisher", rclcpp::ParametersQoS{}, m_param_cbg)}
{
  waitForServices("robot_state_publisher", *m_urdf_client);
  waitForServices("srdf_publisher", *m_srdf_client);
}

std::shared_future<RobotDescription> ParamInterface::loadRobotDescription()
{
  return std::async([this]() {
    const auto urdf = loadParam("robot_description", m_urdf_client);
    const auto srdf = loadParam("robot_description_semantic", m_srdf_client);

    return RobotDescription{urdf.get(), srdf.get()};
  });
}

std::shared_future<void>
ParamInterface::setRobotDescription(const RobotDescription& robot_description)
{
  return std::async([this, robot_description]() {
    const auto set_robot_description =
      setParam("robot_description", robot_description.robot_description, m_urdf_client);
    const auto set_robot_description_semantic = setParam(
      "robot_description_semantic", robot_description.robot_description_semantic, m_srdf_client);

    set_robot_description.get();
    set_robot_description_semantic.get();
  });
}

void ParamInterface::waitForServices(const std::string& name,
                                     rclcpp::AsyncParametersClient& client) const
{
  while (!client.service_is_ready())
  {
    RCLCPP_INFO(m_log, "Waiting for parameter services of %s", name.c_str());
    client.wait_for_service(std::chrono::seconds{3});
  }
}

std::shared_future<std::string>
ParamInterface::loadParam(const std::string& name,
                          const rclcpp::AsyncParametersClient::SharedPtr& client) const
{
  return std::async([this, name, client] {
    const auto params = client->get_parameters({name}).get();
    if (params.size() != 1)
    {
      throw std::runtime_error{
        fmt::format("Incorrect parameter return size {} (expected 1)", params.size())};
    }

    return params[0].get_value<std::string>();
  });
}

std::shared_future<void>
ParamInterface::setParam(const std::string& name,
                         const std::string& value,
                         const rclcpp::AsyncParametersClient::SharedPtr& client) const
{
  return std::async([this, name, value, client]() {
    const auto result = client->set_parameters({rclcpp::Parameter{name, value}}).get();
    if (result.size() != 1)
    {
      throw std::runtime_error{
        fmt::format("Incorrect parameter SetParameterResult size {}, expected 1", result.size())};
    }

    if (!result[0].successful)
    {
      throw std::runtime_error{
        fmt::format("Could not set parameter '{}': {}", name, result[0].reason)};
    }
  });
}

} // namespace tool_change_manager
