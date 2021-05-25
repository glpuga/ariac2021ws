/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <mutex>
#include <string>

// tijcore
#include <tijcore/perception/ModelContainerInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>

namespace tijcore {

// TODO(glpuga) This class needs testing.

class ConveyorBeltModelContainer : public ModelContainerInterface {
public:
  ConveyorBeltModelContainer(
      const std::string &name, const std::string &container_frame_id,
      const std::string &surface_frame_id,
      const std::string &model_tray_shared_access_space_id);

  bool enabled() const override;

  bool isSubmissionTray() const override;

  WorkRegionId region() const override;

  void setEnabled(const bool state) override;
};

} // namespace tijcore
