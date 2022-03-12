/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

namespace tijcore
{
enum class BinId
{
  bin1,
  bin2,
  bin3,
  bin4,
  bin5,
  bin6,
  bin7,
  bin8
};

namespace bin
{
BinId fromString(const std::string& sid);

std::string toString(const BinId& id);

bool isValid(const std::string& sid);

}  // namespace bin

std::ostream& operator<<(std::ostream& os, BinId id);

}  // namespace tijcore
