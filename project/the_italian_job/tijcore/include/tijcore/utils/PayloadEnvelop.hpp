/* Copyright [2022] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <stdexcept>
#include <string>
#include <unordered_map>

// tijcore
#include <tijcore/datatypes/MovableTrayId.hpp>
#include <tijcore/datatypes/PartTypeId.hpp>
#include <tijmath/Position.hpp>
#include <tijmath/Rotation.hpp>

namespace tijcore
{
class PayloadEnvelope
{
public:
  static PayloadEnvelope makeEnvelope(const PartTypeId& part_type_id)
  {
    const auto part_dimensions = dimensions(part_type_id);
    const auto uuu = tijmath::Position{ part_dimensions / 2.0 };
    const auto nnn = tijmath::Position{ -1.0 * part_dimensions / 2.0 };
    return PayloadEnvelope{ uuu, nnn };
  }

  static PayloadEnvelope makeEnvelope(const MovableTrayId& movable_tray_id)
  {
    const auto part_dimensions = dimensions(movable_tray_id);
    const auto uuu = tijmath::Position{ part_dimensions / 2.0 };
    const auto nnn = tijmath::Position{ -1.0 * part_dimensions / 2.0 };
    return PayloadEnvelope{ uuu, nnn };
  }

  const tijmath::Position& posPosPosCorner() const
  {
    return ppp_;
  }

  const tijmath::Position& negNegNegCorner() const
  {
    return nnn_;
  }

  static double offsetToTop(const tijcore::PartTypeId& part_type_id,
                            const tijmath::Rotation& orientation_in_world)
  {
    const auto& rotation_matrix = orientation_in_world.rotationMatrix();
    const auto part_dimensions = dimensions(part_type_id);
    const double estimated_height = std::abs(tijmath::Vector3{ 0.0, 0.0, 1 }.dot(
        part_dimensions[0] * rotation_matrix.col(0) + part_dimensions[1] * rotation_matrix.col(1) +
        part_dimensions[2] * rotation_matrix.col(2)));
    return estimated_height * 0.5;  // part poses are centered in the body
  }

  static double offsetToTop(const tijcore::MovableTrayId&)
  {
    return 0.01;  // tray height at the gripping point is 1 cm
  }

private:
  tijmath::Position ppp_;
  tijmath::Position nnn_;

  PayloadEnvelope(const tijmath::Position& ppp, const tijmath::Position& nnn)
    : ppp_{ ppp }, nnn_{ nnn }
  {
  }

  static tijmath::Vector3 dimensions(const PartTypeId& id)
  {
    static const std::unordered_map<PartTypeId, tijmath::Vector3> id_map = {
      { PartTypeId::battery, tijmath::Vector3{ 0.136, 0.06, 0.05 } },
      { PartTypeId::sensor, tijmath::Vector3{ 0.125, 0.105, 0.06 } * 1.1 },
      { PartTypeId::regulator, tijmath::Vector3{ 0.093, 0.126, 0.066 } },
      { PartTypeId::pump, tijmath::Vector3{ 0.11, 0.11, 0.11 } },
    };
    auto it = id_map.find(id);
    if (it == id_map.end())
    {
      throw std::invalid_argument{ "Invalid part type id" };
    }

    return it->second;
  }

  static tijmath::Vector3 dimensions(const MovableTrayId& id)
  {
    return tijmath::Vector3{ 0.40, 0.60, 0.05 };
  }
};

}  // namespace tijcore
