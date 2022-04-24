/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>

namespace tijcore
{
class PickAndPlaceTask : public RobotTaskInterface
{
public:
  PickAndPlaceTask(const ResourceManagerInterface::SharedPtr& resource_manager,
                   const Toolbox::SharedPtr& toolbox,
                   ResourceManagerInterface::ManagedLocusHandle&& source,
                   ResourceManagerInterface::ManagedLocusHandle&& destination,
                   ResourceManagerInterface::PickAndPlaceRobotHandle&& robot);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  ResourceManagerInterface::SharedPtr resource_manager_;
  Toolbox::SharedPtr toolbox_;
  ResourceManagerInterface::ManagedLocusHandle source_;
  ResourceManagerInterface::ManagedLocusHandle destination_;
  ResourceManagerInterface::PickAndPlaceRobotHandle robot_;
};

}  // namespace tijcore
