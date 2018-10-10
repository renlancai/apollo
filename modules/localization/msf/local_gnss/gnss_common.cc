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

void GnssPntSolver::PrintMatrix(const Eigen::MatrixXd &t, const char *t_name) {
  if (!debug_print_) {
    return;
  }
  gnss_utility::PrintEigenMatrix(t, t_name);
}

double GnssPntSolver::RoundDouble(double x) {
  return static_cast<double>(std::floor(x + 0.5));
}

void GnssPntSolver::AddSolvedBand(const GnssBandID &b_id) {
  if (gnss_utility::GetIndexInVector(b_id, band_to_solve_) == -1) {
    band_to_solve_.push_back(b_id);
  }
}

bool GnssPntSolver::IsGlonassExisted(
    const std::vector<GnssType> &related_gnss_type) {
  unsigned int size = related_gnss_type.size();
  for (unsigned int i = 0; i < size; ++i) {
    if (related_gnss_type[i] == apollo::drivers::gnss::GLO_SYS) {
      return true;
    }
  }
  return false;
}

double GnssPntSolver::GetBandLength(
    const int prn, const apollo::drivers::gnss::GnssBandID &fre_id,
    int* glo_fre_num) {
  return global_ephemeris_.GetBandLength(prn, fre_id, glo_fre_num);
}

void GnssPntSolver::LogFixedAmbToBuffer(const char *amb_log) {
  // TO DO:
}

bool GnssPntSolver::CaculateHeading(const PointThreeDim &ant_base,
                                   const PointThreeDim &ant_slave,
                                   const PointThreeDim &phase_std) {
  // TO DO:
  return true;
}

void GnssPntSolver::EncodeDetails(const char *format, ...) {
  const int buf_size = 2048;
  char temp[buf_size] = {'\0'};
  va_list args;
  va_start(args, format);
  vsprintf(temp, format, args);
  va_end(args);
  invalid_rtk_sub_infor_ = std::string(temp);
}

bool GnssPntSolver::CheckPossibleHalfSlip(const unsigned int loss_lock_index) {
  if (loss_lock_index == 2) {
    EnableHalfCycleAr(true);
  }
  return true;
}

bool GnssPntSolver::ExcludeSemiCycleSat(
    const SatelliteObservation &sat_obs) {
  for (unsigned int i = 0; i < sat_obs.band_obs_num(); ++i) {
    if (!sat_obs.band_obs(i).has_loss_lock_index()) {
      continue;
    }
    if (sat_obs.band_obs(i).loss_lock_index() == 2) {
      return true;
    }
  }
  return false;
}

bool GnssPntSolver::DeleteElvCutoffSat(
    const double ele_cutoff, std::vector<SatelliteInfor>* sate_vector_used) {
  // return true;
  std::vector<SatelliteInfor>::iterator it = sate_vector_used->begin();
  for (; it != sate_vector_used->end();) {
    if (it->elevation < ele_cutoff || it->multi_path == true) {
      it = sate_vector_used->erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

unsigned int GnssPntSolver::SelectCommonSatellite(
    const std::vector<SatelliteInfor> sate_used_rover,
    const std::vector<SatelliteInfor> sate_used_baser,
    const PointThreeDim& baser_coor,
    std::vector<SatelliteInfor>* common_in_rover,
    std::vector<SatelliteInfor>* common_in_baser,
    std::vector<GnssType>* related_gnss_type) {
  common_in_rover->resize(0);
  common_in_baser->resize(0);
  related_gnss_type->resize(0);
  unsigned int common_num = 0;
  for (unsigned int i = 0; i < sate_used_rover.size(); ++i) {
    SatelliteInfor sat_rover = sate_used_rover[i];
    if (gnss_utility::GetIndexInVector(sat_rover.sat_sys, gnss_to_solve_) ==
        -1) {
      continue;
    }
    for (unsigned int j = 0; j < sate_used_baser.size(); ++j) {
      SatelliteInfor sat_baser = sate_used_baser[j];
      if (sat_rover.IsSameSatellite(sat_baser) &&
          sat_rover.elevation > elevation_cutoff_deg_) {
        // update baser distance and direction with baser true coordinate
        sat_baser.distance = gnss_utility::GetCosineDirection(
          baser_coor, sat_baser.position, &sat_baser.direction);
        common_in_baser->push_back(sat_baser);
        common_in_rover->push_back(sat_rover);
        ++common_num;
        // count the number of GNSS system
        if (gnss_utility::GetIndexInVector(sat_rover.sat_sys,
                                          *related_gnss_type) == -1) {
          related_gnss_type->push_back(sat_rover.sat_sys);
        }
        break;
      }
    }
  }
  return common_num;
}

bool GnssPntSolver::FilterOutGnssSystem(
    std::vector<GnssType>* related_gnss_type) {
  std::vector<GnssType>::iterator iter =
      related_gnss_type->begin();
  for (iter = related_gnss_type->begin(); iter != related_gnss_type->end();) {
    if (gnss_utility::GetIndexInVector(*iter, gnss_to_solve_) == -1) {
      iter = related_gnss_type->erase(iter);
    } else {
      ++iter;
    }
  }
  return related_gnss_type->size() > 0;
}

bool GnssPntSolver::GetFixedAmb(const GnssBandID &band_id,
                              const unsigned int sat_prn, double* amb_val,
                              int* index_unfix) {
  if (amb_tracker_.GetFixedAmb(band_id, sat_prn, amb_val)) {
    return true;
  }
  *index_unfix = -1;
  amb_tracker_.GetUnfixedAmbIndex(band_id, sat_prn, index_unfix);
  return false;
}

bool GnssPntSolver::UpdateBandElevation(
    const apollo::drivers::gnss::GnssBandID &band_id,
    const unsigned int sat_prn, const double &elv) {
  return amb_tracker_.SetElevation(band_id, sat_prn, elv);
}

bool GnssPntSolver::IsToughNewSat(const ObsKey &new_sat_obs) {
  std::map<ObsKey, double>::iterator iter =
      tough_fixing_pool_.find(new_sat_obs);
  if (iter == tough_fixing_pool_.end()) {
    return false;
  } else {
    iter->second = iter->second - 1.0;
    if (iter->second <= 0) {
      tough_fixing_pool_.erase(iter);
      return false;
    }
  }
  return true;
}

bool GnssPntSolver::SeedToughNewSat(const ObsKey &new_sat_obs,
                                       double ratio) {
  std::map<ObsKey, double>::iterator iter =
      tough_fixing_pool_.find(new_sat_obs);
  double val = 3.0;
  if (ratio < 2.0) {
    val = 5.0;
  }
  if (iter != tough_fixing_pool_.end()) {
    iter->second = val;
  } else {
    tough_fixing_pool_.insert(
        std::map<ObsKey, double>::value_type(new_sat_obs, val));
  }
  return true;
}

bool GnssPntSolver::SeedToughSatGroup(double ratio) {
  unsigned int size = amb_tracker_.GetNumUnfixedAmb();
  for (unsigned int i = 0; i < size; ++i) {
    ObsKey temp;
    amb_tracker_.GetUnfixedPhase(i, &temp);
    SeedToughNewSat(temp, ratio);
  }
  return true;
}

bool GnssPntSolver::CheckSlipOfUnfixedPhase() {
  if (rover_preprocessor_.GetSlipSize() == 0 &&
      baser_preprocessor_.GetSlipSize() == 0) {
    return true;
  }
  // reset unresolved information
  ClearSequential();
  return false;
}

bool GnssPntSolver::RestoreSDAmb(const Eigen::MatrixXd &float_sd,
                                   Eigen::MatrixXd* integer_sd) {
  return amb_tracker_.UpdateSDAmb(float_sd, integer_sd);
}

bool GnssPntSolver::AddToFixedPool(const Eigen::MatrixXd &integer_sd) {
  return amb_tracker_.AddFixedAmb(integer_sd);
}

bool GnssPntSolver::ClearSequential() {
  btpb_.resize(0, 0);
  btpl_.resize(0, 0);
  float_sd_cvc_.resize(0, 0);
  amb_tracker_.ResetUnfixedPhase();
  return true;
}

Eigen::MatrixXd GnssPntSolver::SequentialLS(const Eigen::MatrixXd &btpb,
                                          const Eigen::MatrixXd &btpl) {
  if (btpb.rows() > btpb_.rows()) {
    // ascending satellite
    Eigen::MatrixXd temp_b = btpb_;
    Eigen::MatrixXd temp_l = btpl_;
    btpb_.resize(btpb.rows(), btpb.cols());
    btpl_.resize(btpl.rows(), btpl.cols());
    btpb_.setZero();
    btpl_.setZero();
    btpb_.topLeftCorner(temp_b.rows(), temp_b.cols()) = temp_b;
    btpl_.topLeftCorner(temp_l.rows(), temp_l.cols()) = temp_l;
  }
  btpb_ += btpb;
  btpl_ += btpl;
  float_sd_cvc_ = btpb_.inverse();
  Eigen::MatrixXd float_amb = float_sd_cvc_ * btpl_;
  return float_amb;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
