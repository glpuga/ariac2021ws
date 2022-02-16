/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>

// tijcore
#include <tijcore/datatypes/RobotTaskOutcome.hpp>

namespace tijcore
{
class RobotTaskInterface
{
public:
  using Ptr = std::unique_ptr<RobotTaskInterface>;
  using SharedPtr = std::shared_ptr<RobotTaskInterface>;

  virtual ~RobotTaskInterface() = default;

  virtual RobotTaskOutcome run() = 0;

  virtual void halt() = 0;
};

}  // namespace tijcore
