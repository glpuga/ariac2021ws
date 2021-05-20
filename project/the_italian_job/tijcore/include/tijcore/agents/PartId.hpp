/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/agents/PartColorId.hpp>
#include <tijcore/agents/PartTypeId.hpp>

namespace tijcore {

class PartId {
public:
  static const PartId UnkownPartId;

  PartId(const std::string &coded_part_str);

  PartId(const PartTypeId &type, const PartColorId &color);

  PartTypeId type() const;

  PartColorId color() const;

  std::string codedString() const;

  bool operator==(const PartId &rhs) const;
  bool operator!=(const PartId &rhs) const;

private:
  PartId() : undefined_{true} {}

  PartTypeId type_;
  PartColorId color_;
  bool undefined_{false};
};

} // namespace tijcore
