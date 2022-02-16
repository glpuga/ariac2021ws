/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <chrono>
#include <string>

// tijcore
#include <tijcore/containers/BinModelContainer.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
const CuboidVolume bin_container_volume_{
  tijmath::Vector3(-0.25, -0.25, -0.1),
  tijmath::Vector3(0.25, 0.25, 0.15),
};

}  // namespace

BinModelContainer::BinModelContainer(const std::string& name, const std::string& local_frame_id,
                                     const WorkRegionId& work_region,
                                     const std::string& model_tray_shared_access_space_id)
  : ModelContainerInterface(name, local_frame_id, local_frame_id, tijmath::RelativePose3{ local_frame_id, {} },
                            bin_container_volume_, model_tray_shared_access_space_id)
  , work_region_{ work_region }
{
}

bool BinModelContainer::enabled() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // bins are always enabled
  return true;
}

bool BinModelContainer::isSubmissionTray() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // bins are never submission trays
  return false;
}

WorkRegionId BinModelContainer::region() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // bin can be located in one region or another
  return work_region_;
}

void BinModelContainer::setEnabled(const bool /*state*/)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  ERROR("{} got called on bin {}, but bins should not be disabled", __PRETTY_FUNCTION__, name());
}

}  // namespace tijcore
