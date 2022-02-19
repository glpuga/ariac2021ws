/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <mutex>

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>

namespace tijcore
{
class RemoveBrokenPartTask : public RobotTaskInterface
{
public:
  RemoveBrokenPartTask(const ResourceManagerInterface::SharedPtr& resource_manager,
                       const Toolbox::SharedPtr& toolbox,
                       ResourceManagerInterface::ManagedLocusHandle&& target,
                       ResourceManagerInterface::PickAndPlaceRobotHandle&& robot);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  ResourceManagerInterface::SharedPtr resource_manager_;
  Toolbox::SharedPtr toolbox_;
  ResourceManagerInterface::ManagedLocusHandle target_;
  ResourceManagerInterface::PickAndPlaceRobotHandle robot_;
};

}  // namespace tijcore
