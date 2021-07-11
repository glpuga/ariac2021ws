/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// gtest
#include "gmock/gmock.h"

// tijcore
#include <tijcore/perception/ModelPerceptionInterface.hpp>

namespace tijcore {

class ModelPerceptionMock : public ModelPerceptionInterface {
public:
  MOCK_CONST_METHOD0(getObservedModels, std::vector<ObservedModel>());
};

} // namespace tijcore
