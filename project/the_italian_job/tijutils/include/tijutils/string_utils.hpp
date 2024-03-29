/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>
#include <vector>

namespace tijutils
{
namespace string_utils
{
std::vector<std::string> splitStringByChar(const std::string& s, const char sep);

std::string joinStringsWithSeparator(const std::vector<std::string>& sv, const char sep);

std::string joinStringsWithSeparator(const std::vector<std::string>& sv, const std::string& sep);

}  // namespace string_utils

}  // namespace tijutils
