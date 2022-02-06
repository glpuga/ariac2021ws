/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <string>

// tijcore
#include <tijcore/logger/logger.hpp>
#include <tijcore/perception/AssemblyStationModelContainer.hpp>

namespace tijcore
{
namespace
{
const CuboidVolume briefcase_container_volume_{
  Vector3(-0.25, -0.25, -0.1),
  Vector3(0.25, 0.25, 0.30),
};

}  // namespace

AssemblyStationModelContainer::AssemblyStationModelContainer(const std::string& name, const std::string& local_frame_id,
                                                             const std::string& model_tray_shared_access_space_id,
                                                             const Toolbox::SharedPtr toolbox)
  : ModelContainerInterface(name, local_frame_id, local_frame_id, RelativePose3{ local_frame_id, {} },
                            briefcase_container_volume_, model_tray_shared_access_space_id)
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

WorkRegionId AssemblyStationModelContainer::region() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // assembly stations always in the assembly region
  return WorkRegionId::assembly;
}

void AssemblyStationModelContainer::setEnabled(const bool state)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  INFO("{} has been enabled (may have been submitted)", name());
  enabled_ = state;
}

}  // namespace tijcore
