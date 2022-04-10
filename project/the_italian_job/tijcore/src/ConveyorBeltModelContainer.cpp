/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <string>

// tijcore
#include <tijcore/containers/ConveyorBeltModelContainer.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
const CuboidVolume bin_container_volume_{
  tijmath::Vector3(-0.32, -100.0, -0.1),
  tijmath::Vector3(0.32, 100.0, 0.20),
};

}  // namespace

ConveyorBeltModelContainer::ConveyorBeltModelContainer(
    const std::string& name, const std::string& container_frame_id,
    const std::string& surface_frame_id, const std::string& model_tray_shared_access_space_id)
  : ModelContainerInterface(name, container_frame_id, surface_frame_id,
                            tijmath::RelativePose3{ container_frame_id, {} }, bin_container_volume_,
                            model_tray_shared_access_space_id)
{
}

bool ConveyorBeltModelContainer::enabled() const
{
  // the conveyor belt is always enabled
  return true;
}

bool ConveyorBeltModelContainer::isSubmissionTray() const
{
  // conveyor belts are not submission trays
  return false;
}

void ConveyorBeltModelContainer::setEnabled(const bool /*state*/)
{
  ERROR(
      "{} got called on the conveyor belt {}, but but it should not be "
      "disabled",
      __PRETTY_FUNCTION__, name());
}

}  // namespace tijcore
