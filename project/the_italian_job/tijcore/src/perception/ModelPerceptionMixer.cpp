/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <utility>

// tijcore
#include <tijcore/perception/ModelPerceptionMixer.hpp>

namespace tijcore
{
ModelPerceptionMixer::ModelPerceptionMixer(std::vector<ModelPerceptionInterface::Ptr>&& children)
  : children_{ std::move(children) }
{
}

std::vector<ObservedModel> ModelPerceptionMixer::getObservedModels() const
{
  std::vector<ObservedModel> models;

  for (const auto& camera : children_)
  {
    auto camera_models = camera->getObservedModels();
    std::move(camera_models.begin(), camera_models.end(), std::back_inserter(models));
  }

  return models;
}

}  // namespace tijcore
