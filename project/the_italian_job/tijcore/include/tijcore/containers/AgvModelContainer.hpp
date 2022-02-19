/* Copyright [2021] <TheItalianJob>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <mutex>
#include <string>

// tijcore
#include <tijcore/abstractions/ModelContainerInterface.hpp>
#include <tijcore/coremodels/Toolbox.hpp>
#include <tijutils/Timer.hpp>

namespace tijcore
{
// TODO(glpuga) This class needs testing.

class AgvModelContainer : public ModelContainerInterface
{
public:
  AgvModelContainer(const std::string& name, const std::string& local_frame_id,
                    const std::string& model_tray_shared_access_space_id,
                    const Toolbox::SharedPtr toolbox);

  bool enabled() const override;

  bool isSubmissionTray() const override;

  WorkRegionId region() const override;

  void setEnabled(const bool state) override;

private:
  mutable std::mutex mutex_;

  Toolbox::SharedPtr toolbox_;

  bool enabled_{ true };

  tijutils::Timer timer_;

  void timerCallback();
};

}  // namespace tijcore
