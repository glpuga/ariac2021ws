// Copyright (2022) Gerardo Puga
// Distributed under the MIT License (http://opensource.org/licenses/MIT)

// Standard library
#include <ostream>
#include <sstream>
#include <string>

// project
#include <tijutils/UniqueId.hpp>

namespace tijutils
{
std::atomic<UniqueId::UType> UniqueId::instance_counter_;

UniqueId UniqueId::CreateNewId()
{
  return UniqueId{ instance_counter_++ };
}

bool UniqueId::operator==(const UniqueId& rhs) const
{
  return id_ == rhs.id_;
}

bool UniqueId::operator!=(const UniqueId& rhs) const
{
  return id_ != rhs.id_;
}

bool UniqueId::operator<(const UniqueId& rhs) const
{
  return id_ < rhs.id_;
}

bool UniqueId::operator>(const UniqueId& rhs) const
{
  return id_ > rhs.id_;
}

bool UniqueId::operator<=(const UniqueId& rhs) const
{
  return (id_ < rhs.id_) || (id_ == rhs.id_);
}

bool UniqueId::operator>=(const UniqueId& rhs) const
{
  return (id_ > rhs.id_) || (id_ == rhs.id_);
}

UniqueId::UType UniqueId::value() const
{
  return id_;
}

UniqueId::UniqueId(const UType new_id) : id_{ new_id }
{
}

namespace unique_id
{
std::string toString(const UniqueId& id)
{
  std::ostringstream os;
  os << std::hex << id.value();
  return os.str();
}

};  // namespace unique_id

std::ostream& operator<<(std::ostream& os, UniqueId id)
{
  return os << unique_id::toString(id);
}

}  // namespace tijutils
