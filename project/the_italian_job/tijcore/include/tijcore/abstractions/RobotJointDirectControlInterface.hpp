/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <map>
#include <memory>
#include <string>

namespace tijcore
{
class RobotJointDirectControlInterface
{
public:
  using Ptr = std::unique_ptr<RobotJointDirectControlInterface>;

  virtual ~RobotJointDirectControlInterface() = default;

  virtual bool setJointPosition(const std::map<std::string, double>& updated_values,
                                const double interval) = 0;

  virtual bool inTrackingMode() const = 0;
};

}  // namespace tijcore
