/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// standard library
#include <chrono>
#include <string>

// tijcore
#include <tijcore/logger/logger.hpp>
#include <tijcore/perception/AgvModelContainer.hpp>

namespace tijcore
{
namespace
{
constexpr std::chrono::seconds agv_status_report_interval_{ 5 };

const CuboidVolume agv_container_volume_{
  Vector3(-0.20, -0.30, -0.1),
  Vector3(0.20, 0.30, 0.20),
};

}  // namespace

AgvModelContainer::AgvModelContainer(const std::string& name, const std::string& local_frame_id,
                                     const std::string& model_tray_shared_access_space_id,
                                     const Toolbox::SharedPtr toolbox)
  : ModelContainerInterface(name, local_frame_id, local_frame_id, RelativePose3{ local_frame_id, {} },
                            agv_container_volume_, model_tray_shared_access_space_id)
  , toolbox_{ toolbox }
  , timer_{ [this] { timerCallback(); } }
{
  timer_.start(std::chrono::seconds{ agv_status_report_interval_ });
}

bool AgvModelContainer::enabled() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  return enabled_;
}

bool AgvModelContainer::isSubmissionTray() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // Agvs are submission trays
  return true;
}

WorkRegionId AgvModelContainer::region() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // agvs can be located in one region or another, even during the same
  // simulation
  auto process_manager = toolbox_->getProcessManager();
  auto current_station = process_manager->getAgvStation(agv::fromString(name()));

  if (station_id::isKittingStation(current_station))
  {
    return WorkRegionId::kitting_agvs;
  }
  return WorkRegionId::assembly;
}

void AgvModelContainer::setEnabled(const bool state)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  INFO("{} has been enabled (may have been submitted)", name());
  enabled_ = state;
}

void AgvModelContainer::timerCallback()
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  auto process_manager = toolbox_->getProcessManager();
  INFO("AGV status for {}: enabled={}, station={}, status={}", name(), enabled_ ? "true" : "false",
       process_manager->getAgvStation(agv::fromString(name())), process_manager->getAgvState(agv::fromString(name())));
}

}  // namespace tijcore
