/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// tijcore
#include <tijcore/agents/PartColorId.hpp>
#include <tijcore/agents/PartTypeId.hpp>

namespace tijcore
{
class OrderId
{
public:
  using OrderIdType = int;

  OrderId() = default;

  OrderId(const std::string& coded_part_str);

  OrderId(const OrderIdType& id, const bool is_update = false);

  OrderIdType id() const
  {
    return id_;
  }

  bool isUpdate() const
  {
    return is_update_;
  }

  std::string codedString() const;

  bool operator==(const OrderId& rhs) const;
  bool operator!=(const OrderId& rhs) const;

private:
  OrderIdType id_;
  bool is_update_{ false };
};

}  // namespace tijcore
