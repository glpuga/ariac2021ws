/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

// tijcore
#include <tijcore/datatypes/PartColorId.hpp>
#include <tijcore/datatypes/PartTypeId.hpp>

namespace tijcore
{
class PartId
{
public:
  static const PartId UnkownPartId;

  explicit PartId(const std::string& coded_part_str);

  explicit PartId(const PartTypeId& type, const PartColorId& color);

  PartTypeId type() const;

  PartColorId color() const;

  std::string codedString() const;

  static bool isValid(const std::string& coded_part_str);

  bool defined() const
  {
    return !undefined_;
  }

  bool operator==(const PartId& rhs) const;
  bool operator!=(const PartId& rhs) const;

private:
  PartId() : undefined_{ true }
  {
  }

  PartTypeId type_;
  PartColorId color_;
  bool undefined_{ false };
};

}  // namespace tijcore
