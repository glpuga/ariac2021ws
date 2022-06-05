/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <string>
#include <utility>

// tijcore
#include <tijcore/abstractions/ConveyorBeltManagerInterface.hpp>
#include <tijcore/abstractions/FrameTransformerInterface.hpp>
#include <tijcore/abstractions/ModelPerceptionInterface.hpp>
#include <tijcore/abstractions/ProcessManagementInterface.hpp>
#include <tijcore/abstractions/RobotActuatorsInterface.hpp>
#include <tijcore/abstractions/SceneConfigReaderInterface.hpp>
#include <tijcore/abstractions/SpatialMutualExclusionManagerInterface.hpp>

namespace tijcore
{
class Toolbox
{
public:
  using SharedPtr = std::shared_ptr<Toolbox>;

  struct Contents
  {
    FrameTransformerInterface::SharedPtr frame_transformer_instance;
    RobotActuatorsInterface::SharedPtr robot_actuator_instance;
    ProcessManagementInterface::SharedPtr process_manager_instance;
    SceneConfigReaderInterface::SharedPtr scene_config_reader_instance;
    SpatialMutualExclusionManagerInterface::SharedPtr spatial_mutual_exclusion_manager;
    ModelPerceptionInterface::SharedPtr unfiltered_model_perception_chain;
    ConveyorBeltManagerInterface::SharedPtr conveyor_belt_manager;
  };

  explicit Toolbox(Contents&& contents) : contents_{ std::move(contents) }
  {
  }

  FrameTransformerInterface::SharedPtr getFrameTransformer() const
  {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__, contents_.frame_transformer_instance);
  }

  RobotActuatorsInterface::SharedPtr getRobotActuator() const
  {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__, contents_.robot_actuator_instance);
  }

  ProcessManagementInterface::SharedPtr getProcessManager() const
  {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__, contents_.process_manager_instance);
  }

  SceneConfigReaderInterface::SharedPtr getSceneConfigReader() const
  {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__, contents_.scene_config_reader_instance);
  }

  SpatialMutualExclusionManagerInterface::SharedPtr getSpatialMutualExclusionManager() const
  {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__,
                                      contents_.spatial_mutual_exclusion_manager);
  }

  ModelPerceptionInterface::SharedPtr getUnfilteredModelPerceptionChain() const
  {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__,
                                      contents_.unfiltered_model_perception_chain);
  }

  ConveyorBeltManagerInterface::SharedPtr getConveyorBeltManager() const
  {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__, contents_.conveyor_belt_manager);
  }

private:
  Contents contents_;

  template <typename T>
  std::shared_ptr<T> returnPtrIfInstanceNotNull(const char* where,
                                                const std::shared_ptr<T>& ptr) const
  {
    if (!ptr)
    {
      throw std::runtime_error{ std::string{ "Attempted to get and instace in " } + where +
                                " but there was none set in the toolbox" };
    }
    return ptr;
  };
};

}  // namespace tijcore
