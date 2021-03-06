/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <mutex>

// tijcore
#include <tijcore/perception/ResourceManagerInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>
#include <tijcore/tasks/RobotTaskInterface.hpp>

namespace tijcore {

class RemoveBrokenPartTask : public RobotTaskInterface {
public:
  RemoveBrokenPartTask(
      const ResourceManagerInterface::SharedPtr &resource_manager,
      const Toolbox::SharedPtr &toolbox,
      ResourceManagerInterface::ManagedLocusHandle &&target,
      ResourceManagerInterface::PickAndPlaceRobotHandle &&robot);

  RobotTaskOutcome run() override;

  void halt() override;

private:
  ResourceManagerInterface::SharedPtr resource_manager_;
  Toolbox::SharedPtr toolbox_;
  ResourceManagerInterface::ManagedLocusHandle target_;
  ResourceManagerInterface::PickAndPlaceRobotHandle robot_;
};

} // namespace tijcore
