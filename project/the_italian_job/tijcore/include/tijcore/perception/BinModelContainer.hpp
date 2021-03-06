/* Copyright [2021] <Ekumen>
 * Author: Gerardo Puga
 */

#pragma once

// standard library
#include <mutex>
#include <string>

// tijcore
#include <tijcore/perception/ModelContainerInterface.hpp>
#include <tijcore/perception/Toolbox.hpp>

namespace tijcore {

// TODO(glpuga) This class needs testing.

class BinModelContainer : public ModelContainerInterface {
public:
  BinModelContainer(const std::string &name, const std::string &local_frame_id,
                    const WorkRegionId &work_region,
                    const std::string &model_tray_shared_access_space_id);

  bool enabled() const override;

  bool isSubmissionTray() const override;

  WorkRegionId region() const override;

  void setEnabled(const bool state) override;

private:
  mutable std::mutex mutex_;

  WorkRegionId work_region_;
};

} // namespace tijcore
