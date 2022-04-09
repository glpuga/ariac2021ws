/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>
#include <vector>

// tijcore
#include <tijcore/datatypes/ObservedItem.hpp>

namespace tijcore
{
class ModelPerceptionInterface
{
public:
  using Ptr = std::unique_ptr<ModelPerceptionInterface>;

  virtual ~ModelPerceptionInterface() = default;
  virtual std::vector<ObservedItem> getObservedModels() const = 0;
};

}  // namespace tijcore
