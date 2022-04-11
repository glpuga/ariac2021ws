/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <utility>
#include <vector>

// tijcore
#include <tijcore/coremodels/ModelPerceptionSpatialFilter.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
ModelPerceptionSpatialFilter::ModelPerceptionSpatialFilter(const Toolbox::SharedPtr& toolbox,
                                                           ModelPerceptionInterface::Ptr child)
  : toolbox_{ toolbox }, child_{ std::move(child) }
{
}

void ModelPerceptionSpatialFilter::addBlindVolumeTracker(
    BlindVolumeTrackerInterface::Ptr new_tracker)
{
  blind_volume_trackers_.emplace_back(std::move(new_tracker));
}

std::vector<ObservedItem> ModelPerceptionSpatialFilter::getObservedModels() const
{
  std::vector<ObservedItem> models;

  auto camera_models = child_->getObservedModels();

  for (const auto& model : camera_models)
  {
    bool within_blind_volume = false;
    for (const auto& blind_volume_tracker : blind_volume_trackers_)
    {
      const auto blind_volume_pose = blind_volume_tracker->pose();
      const auto blind_volume_radious = blind_volume_tracker->radius();
      // convert the model pose to the same frame as the tracked volume
      const auto rel_pose = toolbox_->getFrameTransformer()->transformPoseToFrame(
          model.pose, blind_volume_pose.frameId());

      const auto distance = rel_pose.position().vector().norm();
      if (distance < blind_volume_radious)
      {
        within_blind_volume = true;
        break;
      }
    }
    if (!within_blind_volume)
    {
      models.push_back(model);
    }
  }

  return models;
}

}  // namespace tijcore
