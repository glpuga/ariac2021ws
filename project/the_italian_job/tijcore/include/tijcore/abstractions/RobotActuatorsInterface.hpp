/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>

// project
#include <tijcore/abstractions/RobotJointDirectControlInterface.hpp>
#include <tijcore/datatypes/GripperTypeId.hpp>
namespace tijcore
{
class RobotActuatorsInterface
{
public:
  using Ptr = std::unique_ptr<RobotActuatorsInterface>;
  using SharedPtr = std::shared_ptr<RobotActuatorsInterface>;

  struct ConveyorState
  {
    double power;
    bool enabled;
  };

  struct VacuumGripperState
  {
    bool enabled;
    bool attached;
  };

  struct RobotHealthStatus
  {
    bool kitting_robot_enabled;
    bool assembly_robot_enabled;
  };

  virtual ~RobotActuatorsInterface() = default;

  virtual ConveyorState getConveyorState() const = 0;

  virtual VacuumGripperState getGantryGripperState() const = 0;

  virtual bool setGantryGripperSuction(const bool enable) const = 0;

  virtual bool setGantryGripperToolType(const tijcore::GripperTypeId new_type) const = 0;

  virtual VacuumGripperState getKittingGripperState() const = 0;

  virtual bool setKittingGripperSuction(const bool enable) const = 0;

  virtual bool setGantryTrayLockState(const bool lock_state) const = 0;

  virtual RobotHealthStatus getRobotHealthStatus() const = 0;

  virtual tijcore::GripperTypeId getGantryGripperToolType() const = 0;

  virtual tijcore::RobotJointDirectControlInterface& getKittingJointDirectControlManager() = 0;

  virtual tijcore::RobotJointDirectControlInterface& getGantryJointDirectControlManager() = 0;
};

}  // namespace tijcore
