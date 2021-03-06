/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>
#include <utility>

// tijcore
#include <tijcore/competition/ProcessManagementInterface.hpp>
#include <tijcore/competition/RobotActuatorsInterface.hpp>
#include <tijcore/perception/FrameTransformerInterface.hpp>
#include <tijcore/perception/SceneConfigReaderInterface.hpp>

namespace tijcore {

class Toolbox {
public:
  using SharedPtr = std::shared_ptr<Toolbox>;

  struct Contents {
    FrameTransformerInterface::SharedPtr frame_transformer_instance;
    RobotActuatorsInterface::SharedPtr robot_actuator_instance;
    ProcessManagementInterface::SharedPtr process_manager_instance;
    SceneConfigReaderInterface::SharedPtr scene_config_reader_instance;
  };

  Toolbox(Contents &&contents) : contents_{std::move(contents)} {}

  FrameTransformerInterface::SharedPtr getFrameTransformer() const {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__,
                                      contents_.frame_transformer_instance);
  }

  RobotActuatorsInterface::SharedPtr getRobotActuator() const {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__,
                                      contents_.robot_actuator_instance);
  }

  ProcessManagementInterface::SharedPtr getProcessManager() const {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__,
                                      contents_.process_manager_instance);
  }

  SceneConfigReaderInterface::SharedPtr getSceneConfigReader() const {
    return returnPtrIfInstanceNotNull(__PRETTY_FUNCTION__,
                                      contents_.scene_config_reader_instance);
  }

private:
  Contents contents_;

  template <typename T>
  std::shared_ptr<T>
  returnPtrIfInstanceNotNull(const char *where,
                             const std::shared_ptr<T> &ptr) const {
    if (!ptr) {
      throw std::runtime_error{std::string{"Attempted to get and instace in "} +
                               where +
                               " but there was none set in the toolbox"};
    }
    return ptr;
  };
};

} // namespace tijcore
