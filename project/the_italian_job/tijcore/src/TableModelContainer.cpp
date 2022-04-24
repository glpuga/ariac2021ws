/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <chrono>
#include <string>

// tijcore
#include <tijcore/containers/TableModelContainer.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
const CuboidVolume bin_container_volume_{
  tijmath::Vector3(-0.50, -0.75, 0.8 + 0.45),  // 0.45 is a vertical offset because the table frame
                                               // is below the ground
  tijmath::Vector3(0.50, 0.75, 1.2 + 0.45),    // 0.45 is a vertical offset because the table frame
                                               // is below the ground
};

}  // namespace

TableModelContainer::TableModelContainer(const std::string& name, const std::string& local_frame_id)
  : ModelContainerInterface(name, local_frame_id, local_frame_id,
                            tijmath::RelativePose3{ local_frame_id, {} }, bin_container_volume_)
{
}

bool TableModelContainer::enabled() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // tables are always enabled
  return true;
}

bool TableModelContainer::isSubmissionTray() const
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  // tables are never submission trays
  return false;
}

void TableModelContainer::setEnabled(const bool /*state*/)
{
  std::lock_guard<std::mutex> lock{ mutex_ };
  ERROR("{} got called on table {}, but tables should not be disabled", __PRETTY_FUNCTION__,
        name());
}

}  // namespace tijcore
