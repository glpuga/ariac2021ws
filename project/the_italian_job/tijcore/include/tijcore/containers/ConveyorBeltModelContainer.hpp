/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <mutex>
#include <string>

// tijcore
#include <tijcore/abstractions/ModelContainerInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>

namespace tijcore
{
// TODO(glpuga) This class needs testing.

class ConveyorBeltModelContainer : public ModelContainerInterface
{
public:
  ConveyorBeltModelContainer(const std::string& name, const std::string& container_frame_id,
                             const std::string& surface_frame_id);

  bool enabled() const override;

  bool isSubmissionTray() const override;

  void setEnabled(const bool state) override;
};

}  // namespace tijcore
