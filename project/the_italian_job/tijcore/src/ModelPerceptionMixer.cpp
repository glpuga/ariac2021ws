/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <utility>
#include <vector>

// tijcore
#include <tijcore/coremodels/ModelPerceptionMixer.hpp>

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
