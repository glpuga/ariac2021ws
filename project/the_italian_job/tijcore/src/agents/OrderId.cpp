/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <cmath>
#include <stdexcept>
#include <string>

// tijcore
#include <tijcore/agents/OrderId.hpp>
#include <tijcore/utils/string.hpp>

namespace tijcore
{
namespace
{
constexpr char const* coded_order_string_prefix_{ "order" };
constexpr char const* coded_order_string_update_posfix_{ "update" };
constexpr char coded_part_string_separator_{ '_' };

}  // namespace

OrderId::OrderId(const std::string& coded_part_str)
{
  auto tokens = utils::string::splitStringByChar(coded_part_str, coded_part_string_separator_);

  const bool valid_regular_order = (tokens.size() == 2) && (tokens[0] == coded_order_string_prefix_);
  const bool valid_order_update = (tokens.size() == 3) && (tokens[0] == coded_order_string_prefix_) &&
                                  (tokens[2] == coded_order_string_update_posfix_);

  if (!(valid_regular_order || valid_order_update))
  {
    throw std::invalid_argument{ coded_part_str + " is not a valid coded order descriptor" };
  }

  id_ = std::stoi(tokens[1]);
  is_update_ = valid_order_update;

  if ((id_ < 0) || coded_part_str != codedString())
  {
    throw std::invalid_argument{ coded_part_str + " has an invalid order sequence number: " };
  }
}

OrderId::OrderId(const OrderIdType& id, const bool is_update) : id_{ id }, is_update_{ is_update }
{
  if (id_ < 0)
  {
    throw std::invalid_argument{ std::to_string(id_) + " is an invalid order sequence number: " };
  }
}

std::string OrderId::codedString() const
{
  if (!is_update_)
  {
    return utils::string::joinStringsWithSeparator({ coded_order_string_prefix_, std::to_string(id_) },
                                                   coded_part_string_separator_);
  }

  return utils::string::joinStringsWithSeparator(
      { coded_order_string_prefix_, std::to_string(id_), coded_order_string_update_posfix_ },
      coded_part_string_separator_);
}

bool OrderId::operator==(const OrderId& rhs) const
{
  return id_ == rhs.id_;
}

bool OrderId::operator!=(const OrderId& rhs) const
{
  return !(*this == rhs);
}

}  // namespace tijcore
