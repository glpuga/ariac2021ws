/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <memory>

namespace tijcore
{
class HumanMonitorServiceInterface
{
public:
  using Ptr = std::unique_ptr<HumanMonitorServiceInterface>;

  virtual ~HumanMonitorServiceInterface() = default;
};

}  // namespace tijcore
