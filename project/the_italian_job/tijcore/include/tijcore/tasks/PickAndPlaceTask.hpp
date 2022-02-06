/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/perception/ResourceManagerInterface.hpp>
#include <tijcore/tasks/RobotTaskInterface.hpp>

namespace tijcore
{
class PickAndPlaceTask : public RobotTaskInterface
{
public:
  PickAndPlaceTask(const ResourceManagerInterface::SharedPtr& resource_manager,
                   ResourceManagerInterface::ManagedLocusHandle&& source,
                   ResourceManagerInterface::ManagedLocusHandle&& destination,
                   ResourceManagerInterface::PickAndPlaceRobotHandle&& robot);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  ResourceManagerInterface::SharedPtr resource_manager_;
  ResourceManagerInterface::ManagedLocusHandle source_;
  ResourceManagerInterface::ManagedLocusHandle destination_;
  ResourceManagerInterface::PickAndPlaceRobotHandle robot_;
};

}  // namespace tijcore
