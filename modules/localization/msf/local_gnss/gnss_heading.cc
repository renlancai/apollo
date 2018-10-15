/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/localization/msf/local_gnss/gnss_positioning.h"
#include <Eigen/Eigen>
#include <vector>
#include "modules/localization/msf/local_gnss/atmosphere.h"
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"

namespace apollo {
namespace localization {
namespace local_gnss {

double GnssDulaAntSolver::CorrectYaw(const double& yaw) {
  // input yaw should be positive in east by north
  if (yaw > 2 * PI) {
    return yaw - 2 * PI;
  } else if (yaw < -2 * PI) {
    return yaw + 2 * PI;
  } else {
    return yaw;
  }
}

bool GnssDulaAntSolver::CaculateHeading(const PointThreeDim& ant_base,
                                       const PointThreeDim& ant_slave,
                                       const PointThreeDim& phase_std) {
  if (!dual_antenna_mode_) {
    return false;
  }
  double bs_len = gnss_utility::GetDistance(ant_base, ant_slave);
  if (fabs(bs_len / dual_ant_baseline_len_ - 1.0) / PI * 180.0 >
      max_heading_std_) {
    return false;
  }
  PointThreeDim dxyz(ant_slave.x - ant_base.x, ant_slave.y - ant_base.y,
                     ant_slave.z - ant_base.z);
  PointThreeDim denu = dxyz;
  gnss_utility::dxyz2enu(ant_base, dxyz, &denu);

  // heading
  double yaw = atan2(-1.0 * denu.x, denu.y) - rotation_dual_ant_;
  heading_dual_ant_ = yaw;

  // std of heading
  dxyz = phase_std;
  gnss_utility::dxyz2enu(ant_base, dxyz, &denu);
  double std_level = sqrt(denu.x * denu.x + denu.y * denu.y + denu.z * denu.z);
  std_heading_ = std_level / dual_ant_baseline_len_;

  return true;
}

bool GnssDulaAntSolver::GetHeadingWithDualAntenna(double* heading,
                                                      double* hd_std) {
  *heading = CorrectYaw(-1.0 * heading_dual_ant_ + 2 * PI);
  *hd_std = std_heading_;
  return GetFixStatus();
}

void GnssDulaAntSolver::SetDualAntennaMode(bool dual_antenna_mode) {
  dual_antenna_mode_ = dual_antenna_mode;
}

bool GnssDulaAntSolver::SetDualAntLeverArm(
    const PointThreeDim& ant_primary, const PointThreeDim& ant_secondary) {
  relative_lever_arm_.x = ant_secondary.x - ant_primary.x;
  relative_lever_arm_.y = ant_secondary.y - ant_primary.y;
  relative_lever_arm_.z = ant_secondary.z - ant_primary.z;
  dual_ant_baseline_len_ = 0.0;
  dual_ant_baseline_len_ = gnss_utility::GetDistance(ant_primary,
                          ant_secondary);
  rotation_dual_ant_ =
      atan2(-1.0 * relative_lever_arm_.x, relative_lever_arm_.y);

  // dual antennas located strictly in vertical are not supported
  if (sqrt(relative_lever_arm_.x * relative_lever_arm_.x +
           relative_lever_arm_.y * relative_lever_arm_.y) < 0.1) {
    return false;
  }
  return true;
}

int GnssDulaAntSolver::SolveHeading(
    const EpochObservation& rover_obs,
    GnssPntResult* rover_pnt, double* heading,
    double* hd_std, bool* valid_heading) {
  // To do: cope with rover comes earlier than baser, how to wait?
  strict_synch_ = false;
  int flag_pnt = -1;
  flag_pnt = Solve(rover_obs, rover_pnt);
  // automaticly call update_heading
  GetHeadingWithDualAntenna(heading, hd_std);
  *valid_heading = strict_synch_;
  return flag_pnt;
}

int GnssDulaAntSolver::SolveHeadingWithBaser(
    const EpochObservation& rover_obs,
    const EpochObservation& baser_obs,
    GnssPntResult* rover_pnt, double* heading,
    double* hd_std, bool* valid_heading) {
  // To do: cope with rover comes earlier than baser, how to wait?
  strict_synch_ = false;
  int flag_pnt = -1;
  UpdateBaseCoor(rover_obs, baser_obs);
  flag_pnt = SolveWithBaser(rover_obs, baser_obs, true, rover_pnt);
  // automaticly call update_heading
  GetHeadingWithDualAntenna(heading, hd_std);
  *valid_heading = strict_synch_;
  return flag_pnt;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
