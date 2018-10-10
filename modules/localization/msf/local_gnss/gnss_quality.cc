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
#include <sys/time.h>
#include <Eigen/Eigen>
#include <vector>
#include "modules/localization/msf/local_gnss/atmosphere.h"
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"

namespace apollo {
namespace localization {
namespace local_gnss {

// this file contains lots of quality control strategies learned from field
// datas without too much frigid knowledge.

// pnt result quality validation block
bool GnssPntSolver::IsSafeStandardPointPos(const double std_rover,
                                            const double pdop) {
  if (fabs(std_rover) >= 10.0 || std_rover < 0.0 || fabs(pdop) > 20.0) {
    return false;
  }
  return true;
}

bool GnssPntSolver::IsSafeCodeDiffPos(const double dis, const double std_dif,
                                       const bool valid_delta_pos) {
  if (dis < 10.0 && !valid_delta_pos && fabs(std_dif) < 2.0 &&
      fabs(std_dif) >= 0.2) {
    return true;
  } else {
    return false;
  }
}

bool GnssPntSolver::IsSafeFixedSolution(const unsigned int gnss_type_size,
                                        const unsigned int num_fixed_phase) {
  if (!b_rtkfixed_) {
    return false;
  }
  if (num_fixed_phase <= 5 * gnss_type_size) {
    std::string excute_info = "";
    if (num_fixed_phase <= 4 * gnss_type_size) {
      ResetFixedRtk();
      excute_info = "reset";
    }
    EncodeDetails(
        "fixed_phase_num not enough for safe solution = %3d with num_sys = %3d "
        "%s",
        num_fixed_phase, gnss_type_size, excute_info.c_str());
    return false;
  }
  double leve_dop = sqrt(rtk_dop_phase_.x * rtk_dop_phase_.x +
                         rtk_dop_phase_.y * rtk_dop_phase_.y);
  std::string save_temp = gnss_utility::FormatString(
      " Safe_fixed_phase(3d_dop, level_dop, num, new) = %6.3f%6.3f%3d%3d",
      rtk_dop_phase_.Norm3D(), leve_dop, num_fixed_phase,
      new_fixed_phase_num_);

  if (rtk_dop_phase_.Norm3D() < 4.0 || leve_dop < 2.5) {
    invalid_rtk_sub_infor_ = invalid_rtk_sub_infor_ + save_temp;
    return true;
  }

  std::string rist_temp = gnss_utility::FormatString(
      " Risky_fixed_phase(3d_dop, level_dop, num, new) = %6.3f%6.3f%3d%3d",
      rtk_dop_phase_.Norm3D(), leve_dop, num_fixed_phase,
      new_fixed_phase_num_);

  invalid_rtk_sub_infor_ = invalid_rtk_sub_infor_ + rist_temp;

  return false;
}

double GnssPntSolver::GetStdThresForNewSat(const double new_ratio,
                                                const double std_v) {
  double thres_hold = 0.05;
  if (new_ratio < 3) {
    thres_hold = 0.0001;
  } else if (new_ratio < 5) {
    thres_hold = 0.010;
  } else if (new_ratio < 8) {
    thres_hold = 0.020;
  } else if (new_ratio < 15) {
    thres_hold = 0.025;
  }
  if (std_v < 0.03) {
    thres_hold *= 2.0;
  }
  return thres_hold;
}

void GnssPntSolver::WeightOnPostResidual(
    const double std_v,
    const std::vector<unsigned int>& range_type,
    const Eigen::MatrixXd& post_res,
    Eigen::MatrixXd* px_temp) {
  Eigen::MatrixXd px = *px_temp;
  for (unsigned int i = 0; i < range_type.size(); ++i) {
    if (range_type[i] == 0) {
      if (fabs(post_res(i, 0)) > 20 * std_v) {
        px(i, i) *= 0.01;
      } else if (fabs(post_res(i, 0)) > 15 * std_v) {
        px(i, i) *= 0.3;
      } else if (fabs(post_res(i, 0)) > 10 * std_v) {
        px(i, i) *= 0.7;
      }
    } else {
      if (fabs(post_res(i, 0)) > 4 * std_v) {
        px(i, i) *= 0.1;
      } else if (fabs(post_res(i, 0)) > 3 * std_v) {
        px(i, i) *= 0.3;
      } else if (fabs(post_res(i, 0)) > 2 * std_v) {
        px(i, i) *= 0.7;
      }
    }
  }
  *px_temp = px;
}

void GnssPntSolver::RemoveAmbWithAbnormalRes(
    const std::vector<ObsKey>& phase_recorder, const double& std_v,
    const std::vector<unsigned int>& range_type,
    const Eigen::MatrixXd& post_res,
    const double band_res_sum[apollo::drivers::gnss::GLO_G3]) {
  bool phase_slip_exist = false;
  unsigned int ind_phase = 0;
  for (unsigned int i = 0; i < range_type.size(); ++i) {
    if (range_type[i] == 0) {
      continue;
    }
    if (fabs(post_res(i, 0)) > 5 * std_v || fabs(post_res(i, 0)) >= 0.15) {
      phase_slip_exist = true;
      amb_tracker_.DeleteSlipAmb(phase_recorder[ind_phase].band_id,
                                 phase_recorder[ind_phase].sat_prn);
    }
    ++ind_phase;
  }
  if (!phase_slip_exist) {
    amb_tracker_.ReupdateReferenceAmb(band_res_sum);
  }
}

void GnssPntSolver::BoundRangePrecision(const double std_dif) {
  if (std_dif > range_precision_) {
    range_precision_ = std_dif;
  }
}

PointThreeDim GnssPntSolver::BoundedRtkStd() {
  PointThreeDim phase_std = std_rtk_.SquareRoot();
  if (std_rtk_thres_of_phase_dop_.Norm3D() > std_rtk_.Norm3D()) {
    phase_std = std_rtk_thres_of_phase_dop_.SquareRoot();
  }
  return phase_std;
}

// observations quality control
bool GnssPntSolver::CheckSatObsForSPP(const SatelliteObservation& sat_obs) {
  if (sat_obs.band_obs_num() <= 0) {
    return false;
  }

  // 20804163.233 = 69 ms
  // 36804163.233 = 122.7 ms
  double time_duration = sat_obs.band_obs(0).pseudo_range() / C_MPS;

  if (time_duration < 0.030) {
    return false;
  }
  if (time_duration > 0.200) {
    return false;
  }

  return true;
}

int GnssPntSolver::IsValidBandObs(const BandObservation& band_obs1,
                                  const BandObservation& band_obs2) {
  if (band_obs1.band_id() != band_obs2.band_id()) {
    return -1;
  }
  if (gnss_utility::GetIndexInVector(band_obs1.band_id(), band_to_solve_) ==
      -1) {
    return -1;
  }
  return 0;
}

bool GnssPntSolver::CheckBaserObs(double std, double distance) {
  if (fabs(distance) > 100.0) {
    return false;
  }
  if (fabs(std) > 30.0 || std < 0.0) {
    return false;
  }
  return true;
}

int GnssPntSolver::EnableGlonassIfbEstimated() {
  // return 1 && b_rtkfixed_;
  return 0;
}

double GnssPntSolver::WeightScaleOnGnssSystem(
    const GnssType& type) {
  double scale = 25.0;
  switch (type) {
    case apollo::drivers::gnss::GPS_SYS:
      scale = 1.0;
      break;
    case apollo::drivers::gnss::BDS_SYS:
      scale = 1.0;
      break;
    case apollo::drivers::gnss::GLO_SYS:
      scale = 2.25;
      break;
    default:
      scale = 25.0;
  }
  return scale;
}

void GnssPntSolver::CheckConsecutiveTimeOffset(unsigned int week_num,
                                                  double week_sec) {
  double sec_per_week = SECOND_PER_WEEK;
  double t1 = week_num * sec_per_week + week_sec;
  double t2 = gnss_weeknum_ * sec_per_week + gnss_week_second_s_;
  if (fabs(t1 - t2) > 5.0) {
    ResetFixedRtk();
  }
  gnss_week_second_s_ = week_sec;
  gnss_weeknum_ = week_num;
}

bool GnssPntSolver::IsEnabledCodeDiff(const double& data_outage,
                                     const double& dis_rover2baser) {
  if (fabs(data_outage) > 60.0) {
    return false;
  }
  const double dis_tolerance = 50000;
  if (dis_rover2baser > dis_tolerance) {
    return false;
  }
  return true;
}

bool GnssPntSolver::IsEnabledPhaseDiff(const double& data_outage,
                                      const double& dis_rover2baser) {
  if (fabs(data_outage) > 15.0) {
    return false;
  }
  const double dis_tolerance = 25000;
  if (dis_rover2baser > dis_tolerance) {
    return false;
  }
  return true;
}

bool GnssPntSolver::CheckArResult(const double ratio, const double adop) {
  if (ratio < 0.0) {
    return false;
  }
  if (adop > 30.0) {
    return false;
  }
  return true;
}

bool GnssPntSolver::IsAbnormalFixedSolutionWithNewPhase(
    const bool& is_fixed, const double std_fixed,
    const unsigned int unknown_phase_num) {
  if (unknown_phase_num <= 0) {
    return false;
  }
  if (!is_fixed) {
    return false;
  }
  if (fabs(std_fixed) < 0.3) {
    return false;
  }
  return true;
}

bool GnssPntSolver::IsNeedRecursion(const bool& is_fixed,
                                    const double std_fixed,
                                    const unsigned int unknown_phase_num,
                                    const unsigned int common_phase_num) {
  if (unknown_phase_num > common_phase_num) {
    EncodeDetails("More unknown phase: %d > %d", unknown_phase_num,
                   common_phase_num);
    return true;
  }
  if (IsAbnormalFixedSolutionWithNewPhase(is_fixed, std_fixed,
                                             unknown_phase_num)) {
    // RET_INVALID_RTK case 3
    EncodeDetails("abnormal_fixed_solution %6.3f", _std_v);
    // shadow the new arising satellites, ignore them for some seconds.
    SeedToughSatGroup(1.0);
    return true;
  }
  return false;
}

double GnssPntSolver::BoundRatio(const double& ratio) {
  if (ratio > 999.9) {
    return 1000.0;
  }
  return ratio;
}

bool GnssPntSolver::CheckNewsatSolution(const PointThreeDim& denu,
                                          const double& thres,
                                          const double& dis_time_diff) {
  if (fabs(denu.z) > thres * 2) {
    return false;
  }
  if (fabs(denu.y) > thres) {
    return false;
  }
  if (fabs(denu.x) > thres) {
    return false;
  }
  if (fabs(dis_time_diff) > 0.05) {
    return false;
  }
  return true;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
