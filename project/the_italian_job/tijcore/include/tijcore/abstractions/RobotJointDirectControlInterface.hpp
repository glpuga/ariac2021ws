/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <map>
#include <string>

namespace tijcore
{
class RobotJointDirectControlInterface
{
public:
  virtual ~RobotJointDirectControlInterface() = default;

  virtual bool setJointPosition(const std::map<std::string, double>& updated_values,
                                const double interval) = 0;

  virtual bool inTrackingMode() const = 0;
};

}  // namespace tijcore
