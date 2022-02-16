/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>

namespace tijcore
{
enum class WorkRegionId
{
  conveyor_belt,
  kitting_agvs,
  kitting_near_bins,
  kitting_far_bins,
  assembly
};

namespace work_region
{
WorkRegionId fromString(const std::string& sid);

std::string toString(const WorkRegionId& id);

bool isValid(const std::string& sid);

}  // namespace work_region

std::ostream& operator<<(std::ostream& os, WorkRegionId id);

}  // namespace tijcore
