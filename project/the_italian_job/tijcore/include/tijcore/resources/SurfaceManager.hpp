/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <optional>
#include <random>
#include <tuple>
#include <vector>

namespace tijcore
{
class SurfaceManager
{
public:
  SurfaceManager(const double x0, const double y0, const double width, const double height);

  void markAsOccupied(const double x, const double y, const double radius);

  bool regionIsAvailable(const double x, const double y, const double radius) const;

  bool regionIsWithinSurface(const double x, const double y, const double radius) const;

  std::optional<std::tuple<double, double>> findFreeRegion(const double free_radius) const;

private:
  struct RegionInUse
  {
    double x;
    double y;
    double radius;
  };

  double x0_{ 0.0 };
  double y0_{ 0.0 };
  double width_{ 0.0 };
  double height_{ 0.0 };

  std::vector<RegionInUse> regions_in_use_;

  mutable std::mt19937 random_generator_;  // started with a default seed,
                                           // provides determinism to tests

  std::optional<double> minimumDistanceToOthers(const RegionInUse& candidate) const;

  bool regionIsAvailable(const RegionInUse& candidate) const;

  bool regionIsWithinSurface(const RegionInUse& candidate) const;

  static bool regionsOverlap(const RegionInUse& region_a, const RegionInUse& region_b);

  static double distanceBetweenRegions(const RegionInUse& region_a, const RegionInUse& region_b);
};

}  // namespace tijcore
