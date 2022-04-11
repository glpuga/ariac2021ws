/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <vector>

// tijcore
#include <tijcore/abstractions/BlindVolumeTrackerInterface.hpp>
#include <tijcore/abstractions/ModelPerceptionInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>

namespace tijcore
{
class ModelPerceptionSpatialFilter : public ModelPerceptionInterface
{
public:
  explicit ModelPerceptionSpatialFilter(const Toolbox::SharedPtr& toolbox,
                                        ModelPerceptionInterface::Ptr child);

  void addBlindVolumeTracker(BlindVolumeTrackerInterface::Ptr new_tracker);

  std::vector<ObservedItem> getObservedModels() const override;

private:
  Toolbox::SharedPtr toolbox_;
  ModelPerceptionInterface::Ptr child_;
  std::vector<BlindVolumeTrackerInterface::Ptr> blind_volume_trackers_;
};

}  // namespace tijcore
