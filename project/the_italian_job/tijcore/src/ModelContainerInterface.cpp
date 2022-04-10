/* Copyright [2021] <TheItalianJob>
 * Distributed under the MIT License (http://opensource.org/licenses/MIT)
 * Author: Gerardo Puga */

// Standard library
#include <string>

// tijcore
#include <tijcore/abstractions/ModelContainerInterface.hpp>

namespace tijcore
{
ModelContainerInterface::ModelContainerInterface(const std::string& name,
                                                 const std::string& container_reference_frame_id,
                                                 const std::string& surface_reference_frame_id,
                                                 const tijmath::RelativePose3& pose,
                                                 const CuboidVolume& container_volume)
  : name_{ name }
  , container_reference_frame_id_{ container_reference_frame_id }
  , surface_reference_frame_id_{ surface_reference_frame_id }
  , pose_{ pose }
  , container_volume_{ container_volume }
{
}

std::string ModelContainerInterface::name() const
{
  return name_;
}

std::string ModelContainerInterface::containerReferenceFrameId() const
{
  return container_reference_frame_id_;
}

std::string ModelContainerInterface::surfaceReferenceFrameId() const
{
  return surface_reference_frame_id_;
}

tijmath::RelativePose3 ModelContainerInterface::pose() const
{
  return pose_;
};

CuboidVolume ModelContainerInterface::containerVolume() const
{
  return container_volume_;
}

}  // namespace tijcore
