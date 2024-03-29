/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// tijcore
#include <tijcore/abstractions/ResourceManagerInterface.hpp>
#include <tijcore/abstractions/RobotTaskInterface.hpp>

namespace tijcore
{
class PickAndTwistPartTask : public RobotTaskInterface
{
public:
  PickAndTwistPartTask(const ResourceManagerInterface::SharedPtr& resource_manager,
                       ResourceManagerInterface::ManagedLocusHandle&& target,
                       ResourceManagerInterface::ManagedLocusHandle&& destination,
                       ResourceManagerInterface::PickAndPlaceRobotHandle&& robot);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  ResourceManagerInterface::SharedPtr resource_manager_;

  ResourceManagerInterface::ManagedLocusHandle target_;
  // notice that we don't need the destination, since we flip the part in place,
  // but keep the managed locus around to avoid some other robot getting the
  // task to move part into that place, resulting in two robots flipping parts
  // for the same target
  ResourceManagerInterface::ManagedLocusHandle destination_;

  ResourceManagerInterface::PickAndPlaceRobotHandle robot_;
};

}  // namespace tijcore
