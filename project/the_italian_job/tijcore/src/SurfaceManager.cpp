/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <cmath>
#include <iostream>
#include <tuple>

// tijcore
#include <tijcore/resources/SurfaceManager.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
namespace
{
static constexpr int32_t max_find_attempts_{ 1000 };
}

SurfaceManager::SurfaceManager(const double x0, const double y0, const double width,
                               const double height)
  : x0_{ x0 }, y0_{ y0 }, width_{ width }, height_{ height }
{
}

void SurfaceManager::markAsOccupied(const double x, const double y, const double radius)
{
  regions_in_use_.push_back(RegionInUse{ x, y, radius });
}

bool SurfaceManager::regionIsAvailable(const double x, const double y, const double radius) const
{
  RegionInUse candidate{ x, y, radius };
  return regionIsAvailable(candidate);
}

bool SurfaceManager::regionIsAvailable(const RegionInUse& candidate) const
{
  if (!regionIsWithinSurface(candidate))
  {
    /* LOG WARNING a region outside of the surface is being asked about */
    return false;
  }
  bool retval{ true };
  for (const auto& other_region : regions_in_use_)
  {
    if (regionsOverlap(candidate, other_region))
    {
      retval = false;
      break;
    }
  }
  return retval;
}

bool SurfaceManager::regionIsWithinSurface(const double x, const double y,
                                           const double radius) const
{
  RegionInUse candidate{ x, y, radius };
  return regionIsWithinSurface(candidate);
}

bool SurfaceManager::regionIsWithinSurface(const RegionInUse& candidate) const
{
  return ((candidate.x - candidate.radius) >= x0_) &&
         ((candidate.x + candidate.radius) <= (width_ + x0_)) &&
         ((candidate.y - candidate.radius) >= y0_) &&
         ((candidate.y + candidate.radius) <= (height_ + y0_));
}

std::optional<std::tuple<double, double>>
SurfaceManager::findFreeRegion(const double free_radius) const
{
  std::optional<RegionInUse> opt_best_candidate;

  std::uniform_real_distribution<> distribution_x(0.0, width_);
  std::uniform_real_distribution<> distribution_y(0.0, height_);

  RegionInUse candidate;
  candidate.radius = free_radius;

  if (!regions_in_use_.empty())
  {
    double current_max_distance{ -1.0 };
    for (size_t attempt = 0; attempt < max_find_attempts_; ++attempt)
    {
      // randomly select a pair of coordinates
      candidate.x = distribution_x(random_generator_) + x0_;
      candidate.y = distribution_y(random_generator_) + y0_;

      if (!regionIsWithinSurface(candidate))
      {
        continue;
      }

      // check if the candidate
      auto opt_distance = minimumDistanceToOthers(candidate);

      if (opt_distance && (current_max_distance < opt_distance.value()))
      {
        current_max_distance = opt_distance.value();
        opt_best_candidate = candidate;
      }
    }
  }
  else
  {
    // there are no other regions in use. Return a deterministically chosen one
    opt_best_candidate = RegionInUse{ x0_ + free_radius, y0_ + free_radius, free_radius };
  }

  if (!opt_best_candidate || !regionIsWithinSurface(opt_best_candidate.value()) ||
      !regionIsAvailable(opt_best_candidate.value()))
  {
    return std::nullopt;
  }

  return std::make_tuple(opt_best_candidate.value().x, opt_best_candidate.value().y);
}

std::optional<double> SurfaceManager::minimumDistanceToOthers(const RegionInUse& candidate) const
{
  if (!regionIsWithinSurface(candidate))
  {
    /* LOG WARNING a region outside of the surface is being asked about */
    return std::nullopt;
  }
  std::optional<double> min_distance{ std::nullopt };
  for (const auto other_region : regions_in_use_)
  {
    auto distance = distanceBetweenRegions(candidate, other_region);
    if (!min_distance || (min_distance.value() > distance))
    {
      min_distance = distance;
    }
  }
  return min_distance;
}

double SurfaceManager::distanceBetweenRegions(const RegionInUse& region_a,
                                              const RegionInUse& region_b)
{
  const double dx = region_a.x - region_b.x;
  const double dy = region_a.y - region_b.y;
  const double distance = std::sqrt(dx * dx + dy * dy);
  return distance;
}

bool SurfaceManager::regionsOverlap(const RegionInUse& region_a, const RegionInUse& region_b)
{
  return (distanceBetweenRegions(region_a, region_b) < (region_a.radius + region_b.radius));
}

}  // namespace tijcore
