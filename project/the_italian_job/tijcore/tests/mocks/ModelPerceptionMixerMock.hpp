/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <vector>

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/abstractions/ModelPerceptionInterface.hpp>

namespace tijcore
{
class ModelPerceptionMock : public ModelPerceptionInterface
{
public:
  MOCK_CONST_METHOD0(getObservedModels, std::vector<ObservedModel>());
};

}  // namespace tijcore
