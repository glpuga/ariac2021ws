/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <memory>
#include <string>

// tijcore
#include <tijcore/abstractions/ModelContainerInterface.hpp>
#include <tijcore/resources/ResourceHandle.hpp>
#include <tijlogger/logger.hpp>

namespace tijcore
{
class SubmissionTrayAdapter
{
public:
  using SharedPtr = std::shared_ptr<SubmissionTrayAdapter>;

  using ModelContainerHandle = ResourceHandle<ModelContainerInterface>;

  explicit SubmissionTrayAdapter(const ModelContainerHandle& container_handle)
    : container_handle_{ container_handle }
  {
  }

  std::string name() const
  {
    return container_handle_.resource()->name();
  }

  tijmath::RelativePose3 pose() const
  {
    return container_handle_.resource()->pose();
  }

  void submit()
  {
    // TODO(glpuga) this can be vastly improved
    if (container_handle_.resource()->isSubmissionTray())
    {
      container_handle_.resource()->setEnabled(false);
    }
    else
    {
      ERROR("Attempted to submit {}, which is not a submission tray");
    }
  }

private:
  ModelContainerHandle container_handle_;
};

}  // namespace tijcore
