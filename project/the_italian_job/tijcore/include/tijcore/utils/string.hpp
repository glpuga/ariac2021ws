/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <string>
#include <vector>

namespace tijcore {

namespace utils {

namespace string {

std::vector<std::string> splitStringByChar(const std::string &s,
                                           const char sep);

std::string joinStringsWithSeparator(const std::vector<std::string> &sv,
                                     const char sep);

std::string joinStringsWithSeparator(const std::vector<std::string> &sv,
                                     const std::string &sep);

} // namespace string

} // namespace utils

} // namespace tijcore
