/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <vector>

// tijcore

#include <tijcore/abstractions/ModelPerceptionInterface.hpp>

namespace tijcore
{
class ModelPerceptionMixer : public ModelPerceptionInterface
{
public:
  explicit ModelPerceptionMixer(std::vector<ModelPerceptionInterface::Ptr>&& children);

  std::vector<ObservedItem> getObservedModels() const override;

private:
  std::vector<ModelPerceptionInterface::Ptr> children_;
};

}  // namespace tijcore
