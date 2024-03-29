/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <string>

// tijcore
#include <tijcore/containers/AssemblyStationModelContainer.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
const CuboidVolume briefcase_container_volume_{
  tijmath::Vector3(-0.25, -0.25, -0.1),
  tijmath::Vector3(0.45, 0.25, 0.30),
};

}  // namespace

AssemblyStationModelContainer::AssemblyStationModelContainer(const std::string& name,
                                                             const std::string& local_frame_id,
                                                             const Toolbox::SharedPtr toolbox)
  : ModelContainerInterface(name, local_frame_id, local_frame_id,
                            tijmath::RelativePose3{ local_frame_id, {} },
                            briefcase_container_volume_)
  , toolbox_{ toolbox }
{
}

bool AssemblyStationModelContainer::enabled() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return enabled_;
}

bool AssemblyStationModelContainer::isSubmissionTray() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // Assembly Stations are submission trays
  return true;
}

void AssemblyStationModelContainer::setEnabled(const bool state)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  INFO("{} has been enabled (may have been submitted)", name());
  enabled_ = state;
}

}  // namespace tijcore
