/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

// Standard library
#include <string>

// tijcore
#include <tijcore/perception/ModelContainerInterface.hpp>

namespace tijcore
{
ModelContainerInterface::ModelContainerInterface(const std::string& name,
                                                 const std::string& container_reference_frame_id,
                                                 const std::string& surface_reference_frame_id,
                                                 const RelativePose3& pose, const CuboidVolume& container_volume,
                                                 const std::string& shared_workspace_id)
  : name_{ name }
  , container_reference_frame_id_{ container_reference_frame_id }
  , surface_reference_frame_id_{ surface_reference_frame_id }
  , pose_{ pose }
  , container_volume_{ container_volume }
  , shared_workspace_id_{ shared_workspace_id }
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

RelativePose3 ModelContainerInterface::pose() const
{
  return pose_;
};

CuboidVolume ModelContainerInterface::containerVolume() const
{
  return container_volume_;
}

std::string ModelContainerInterface::exclusionZoneId() const
{
  return shared_workspace_id_;
}

}  // namespace tijcore
