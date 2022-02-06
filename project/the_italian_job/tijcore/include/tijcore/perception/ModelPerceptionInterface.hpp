/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <vector>

// tijcore
#include <tijcore/agents/ObservedModel.hpp>

namespace tijcore
{
class ModelPerceptionInterface
{
public:
  using Ptr = std::unique_ptr<ModelPerceptionInterface>;
  using SharedPtr = std::shared_ptr<ModelPerceptionInterface>;

  virtual ~ModelPerceptionInterface() = default;
  virtual std::vector<ObservedModel> getObservedModels() const = 0;
};

}  // namespace tijcore
