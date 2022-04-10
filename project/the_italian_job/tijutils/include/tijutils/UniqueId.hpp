// Copyright (2022) Gerardo Puga
// Distributed under the MIT License (http://opensource.org/licenses/MIT)

#pragma once

// standard library
#include <cstdint>
#include <functional>
#include <ostream>
#include <string>

namespace tijutils
{
class UniqueId
{
public:
  using UType = uint64_t;

  /** \brief Static function to create a new unique id number */
  static UniqueId CreateNewId();

  UniqueId(const UniqueId&) = default;
  UniqueId(UniqueId&&) = default;
  UniqueId& operator=(const UniqueId&) = default;
  UniqueId& operator=(UniqueId&&) = default;

  bool operator==(const UniqueId& rhs) const;
  bool operator!=(const UniqueId& rhs) const;
  bool operator<(const UniqueId& rhs) const;
  bool operator>(const UniqueId& rhs) const;
  bool operator<=(const UniqueId& rhs) const;
  bool operator>=(const UniqueId& rhs) const;

  /** \brief Debug method to return the underlying integer that represents the id. Don't rely on
   *         this method for anything else.
   *  \return Implementation-defined integer type representing the type. */
  UType value() const;

private:
  UType id_;
  static UType instance_counter_;

  explicit UniqueId(const UType new_id);
};

namespace unique_id
{
std::string toString(const UniqueId& id);

}  // namespace unique_id

std::ostream& operator<<(std::ostream& os, UniqueId id);

}  // namespace tijutils

template <>
struct std::hash<tijutils::UniqueId>
{
  std::size_t operator()(const tijutils::UniqueId& s) const noexcept
  {
    return std::hash<decltype(s.value())>{}(s.value());
  }
};
