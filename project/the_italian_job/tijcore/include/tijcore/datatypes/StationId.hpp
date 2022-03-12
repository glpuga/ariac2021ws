/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

#pragma once

// standard library
#include <string>

namespace tijcore
{
// TODO(glpuga) this is not a station id, is a location id. station ids are only
// as1, as2, as3, as4, any
enum class StationId
{
  ks1,
  ks2,
  ks3,
  ks4,
  as1,
  as2,
  as3,
  as4,
  any
};

namespace station_id
{
StationId fromString(const std::string& sid);

std::string toString(const StationId& id);

bool isKittingStation(const StationId& id);

bool isAssemblyStation(const StationId& id);

bool isAny(const StationId& id);

bool isValid(const std::string& sid);

}  // namespace station_id

std::ostream& operator<<(std::ostream& os, StationId id);

}  // namespace tijcore
