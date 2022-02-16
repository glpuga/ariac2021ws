/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <stdexcept>
#include <string>

// tijcore
#include <tijcore/datatypes/PartId.hpp>
#include <tijcore/utils/string.hpp>

namespace tijcore
{
namespace
{
constexpr char const* coded_part_string_prefix_{ "assembly" };
constexpr char coded_part_string_separator_{ '_' };

}  // namespace

const PartId PartId::UnkownPartId = PartId{};

PartId::PartId(const std::string& coded_part_str)
{
  auto tokens = utils::string::splitStringByChar(coded_part_str, coded_part_string_separator_);

  if ((tokens.size() != 3) || tokens[0] != coded_part_string_prefix_ || !part_type::isValid(tokens[1]) ||
      !part_color::isValid(tokens[2]))
  {
    throw std::invalid_argument{ coded_part_str + " is not a valid coded part descriptor" };
  }

  type_ = part_type::fromString(tokens[1]);
  color_ = part_color::fromString(tokens[2]);
}

PartId::PartId(const PartTypeId& type, const PartColorId& color) : type_{ type }, color_{ color }
{
}

PartTypeId PartId::type() const
{
  if (undefined_)
  {
    throw std::logic_error("An unknown part id has no defined type");
  }
  return type_;
}

PartColorId PartId::color() const
{
  if (undefined_)
  {
    throw std::logic_error("An unknown part id has no defined color");
  }
  return color_;
}

std::string PartId::codedString() const
{
  if (undefined_)
  {
    throw std::logic_error("An unknown part id has no defined coded string");
  }
  return utils::string::joinStringsWithSeparator(
      { coded_part_string_prefix_, part_type::toString(type_), part_color::toString(color_) },
      coded_part_string_separator_);
}

bool PartId::operator==(const PartId& rhs) const
{
  if (undefined_ && rhs.undefined_)
  {
    return true;
  }
  if (undefined_ != rhs.undefined_)
  {
    return false;
  }
  // the remaining case is none of them is undefined
  return (type_ == rhs.type_) && (color_ == rhs.color_);
}

bool PartId::operator!=(const PartId& rhs) const
{
  return !(*this == rhs);
}

}  // namespace tijcore
