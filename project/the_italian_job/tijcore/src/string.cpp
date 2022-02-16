/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library

// tijcore
#include <tijcore/utils/string.hpp>

// standard library
#include <string>
#include <vector>

namespace tijcore
{
namespace utils
{
namespace string
{
std::vector<std::string> splitStringByChar(const std::string& s, const char sep)
{
  std::vector<std::string> tokens_vector;
  std::string token;
  for (const auto item : s)
  {
    if (item == sep)
    {
      tokens_vector.push_back(token);
      token.clear();
    }
    else
    {
      token.push_back(item);
    }
  }
  if (!s.empty() && *s.end() != sep)
  {
    tokens_vector.push_back(token);
  }
  return tokens_vector;
}

std::string joinStringsWithSeparator(const std::vector<std::string>& sv, const char sep)
{
  return joinStringsWithSeparator(sv, std::string{ sep });
}

std::string joinStringsWithSeparator(const std::vector<std::string>& sv, const std::string& sep)
{
  std::string retval;
  for (auto it = sv.cbegin(); it != sv.cend(); ++it)
  {
    if (it != sv.cbegin())
    {
      retval += sep;
    }
    retval += *it;
  }
  return retval;
}

}  // namespace string

}  // namespace utils

}  // namespace tijcore
