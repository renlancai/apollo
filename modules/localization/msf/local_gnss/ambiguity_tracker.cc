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

#include "modules/localization/msf/local_gnss/ambiguity_tracker.h"
#include "modules/localization/msf/local_gnss/pvt_satellite.h"

namespace apollo {
namespace localization {
namespace local_gnss {

AmbiguityTracker::AmbiguityTracker() { ClearAll(); }

AmbiguityTracker::~AmbiguityTracker() {}

bool AmbiguityTracker::GetLog(
    const apollo::drivers::gnss::GnssBandID &sat_band_id,
    const unsigned int sat_prn, const AmbLogger &logger, double* val) {
  const AmbKey temp(sat_band_id, sat_prn);
  std::map<AmbKey, double>::const_iterator iter = logger.find(temp);
  if (iter != logger.end()) {
    *val = iter->second;
    return true;
  }
  *val = 0.0;
  return false;
}

bool AmbiguityTracker::SetLog(
    const apollo::drivers::gnss::GnssBandID &sat_band_id,
    const unsigned int sat_prn, const double &val, AmbLogger* logger) {
  const AmbKey temp(sat_band_id, sat_prn);
  std::map<AmbKey, double>::iterator iter = logger->find(temp);
  if (iter != logger->end()) {
    iter->second = val;
    return true;
  } else {
    logger->insert(std::map<AmbKey, double>::value_type(temp, val));
    return true;
  }
  return false;
}

bool AmbiguityTracker::DeleteLog(
    const apollo::drivers::gnss::GnssBandID &sat_band_id,
    const unsigned int sat_prn, AmbLogger* logger) {
  const AmbKey temp(sat_band_id, sat_prn);
  std::map<AmbKey, double>::iterator iter = logger->find(temp);
  if (iter != logger->end()) {
    logger->erase(iter);
    return true;
  } else {
    return true;
  }
  return false;
}

bool AmbiguityTracker::GetFixedAmb(
    const apollo::drivers::gnss::GnssBandID &sat_band_id,
    const unsigned int sat_prn, double* ambiguity_val) {
  return GetLog(sat_band_id, sat_prn, amb_fixed_, ambiguity_val);
}

unsigned int AmbiguityTracker::GetNumFixedAmb() { return amb_fixed_.size(); }

unsigned int AmbiguityTracker::GetNumUnfixedAmb() {
  return unresolved_phase_.size();
}

void AmbiguityTracker::ResetUnfixedPhase() { unresolved_phase_.resize(0); }

bool AmbiguityTracker::GetUnfixedPhase(
    unsigned int index, apollo::drivers::gnss::GnssBandID* band_id,
    unsigned int* sat_prn) {
  if (index >= unresolved_phase_.size()) {
    return false;
  }
  *band_id = unresolved_phase_[index].band_id;
  *sat_prn = unresolved_phase_[index].sat_prn;
  return true;
}

bool AmbiguityTracker::GetUnfixedPhase(unsigned int index, ObsKey* sat_obs) {
  if (index >= unresolved_phase_.size()) {
    return false;
  }
  *sat_obs = unresolved_phase_[index];
  return true;
}

void AmbiguityTracker::ClearObsHistory() { obs_elevation_deg_.clear(); }

bool AmbiguityTracker::ClearAll() {
  amb_fixed_.clear();
  unresolved_phase_.clear();
  reference_sat_.clear();
  obs_elevation_deg_.clear();
  half_cycle_phase_.clear();
  return true;
}

// half cycle, not supported in apollo
void AmbiguityTracker::ClearHalfCycleRecorder() {
  half_cycle_phase_.resize(0);
}

void AmbiguityTracker::AddHalfCycleRecorder(const AmbKey &half_cycle_phase) {
  half_cycle_phase_.push_back(half_cycle_phase);
}

bool AmbiguityTracker::IsHalpCycle(const AmbKey &half_cycle_phase) {
  for (unsigned int i = 0; i < half_cycle_phase_.size(); ++i) {
    if (half_cycle_phase_[i] == half_cycle_phase) {
      return true;
    }
  }
  return false;
}

bool AmbiguityTracker::GetElevation(
    const apollo::drivers::gnss::GnssBandID &sat_band_id,
    const unsigned int sat_prn, double* elv_deg) {
  bool b_flag = GetLog(sat_band_id, sat_prn, obs_elevation_deg_, elv_deg);
  if (b_flag == false) {
    *elv_deg = -0.1;
  }
  return b_flag;
}

double AmbiguityTracker::GetElevation(const AmbKey obs_key) {
  double elv_deg = 0.0;
  GetElevation(obs_key.band_id, obs_key.sat_prn, &elv_deg);
  return elv_deg;
}

bool AmbiguityTracker::SetElevation(
    const apollo::drivers::gnss::GnssBandID &sat_band_id,
    const unsigned int sat_prn, const double &elv_deg) {
  return SetLog(sat_band_id, sat_prn, elv_deg, &obs_elevation_deg_);
}

bool AmbiguityTracker::GetUnfixedAmbIndex(
    const apollo::drivers::gnss::GnssBandID &sat_band_id,
    const unsigned int sat_prn, int* index) {
  AmbKey temp(sat_band_id, sat_prn);
  unsigned int size = unresolved_phase_.size();
  for (unsigned int ind = 0; ind < size; ++ind) {
    if (temp == unresolved_phase_[ind]) {
      *index = ind;
      return true;
    }
  }
  // add new unfixed carrier phase
  unresolved_phase_.push_back(temp);
  *index = size;
  return false;
}

int AmbiguityTracker::GetReferenceIndex(AmbKey obs) {
  AmbKey obs_reference;
  bool flag_exist_reference = false;
  // related reference satellite
  unsigned int size0 = reference_sat_.size();
  for (unsigned int i = 0; i < size0; ++i) {
    obs_reference = reference_sat_[i];
    if (obs == obs_reference) {
      return -1;
    }
    if (obs.band_id == obs_reference.band_id) {
      flag_exist_reference = true;
      break;
    }
  }
  unsigned int size1 = unresolved_phase_.size();
  if (!flag_exist_reference) {
    // when the obs lacks of reference obs, reset a reference band obs
    double max_elv_deg = 0.01;
    double elv_deg = 0.0;
    int ind = -1;
    // check out a fixed band satellite as reference
    std::map<AmbKey, double>::iterator iter = amb_fixed_.begin();
    for (; iter != amb_fixed_.end(); ++iter) {
      AmbKey temp = iter->first;
      if (obs.band_id != temp.band_id) {
        continue;
      }
      elv_deg = GetElevation(temp);
      if (elv_deg > max_elv_deg) {
        max_elv_deg = elv_deg;
        obs_reference = temp;
        ind = -2;
      }
    }
    // when fixed amb unavailable, choose one from unfixed
    if (ind == -1) {
      for (unsigned int i = 0; i < size1; ++i) {
        if (obs.band_id != unresolved_phase_[i].band_id) {
          continue;
        }
        elv_deg = GetElevation(unresolved_phase_[i]);
        if (elv_deg > max_elv_deg) {
          max_elv_deg = elv_deg;
          obs_reference = unresolved_phase_[i];
          ind = i;
        }
      }
    }
    reference_sat_.push_back(obs_reference);
    // only the obs itself being set as reference obs
    if (obs == obs_reference) {
      return -1;
    }
    if (ind != -1) {
      return ind;
    }
  }
  // index
  for (unsigned int i = 0; i < size1; ++i) {
    if (obs_reference == unresolved_phase_[i]) {
      return i;
    }
  }
  return -2;
}

bool AmbiguityTracker::GetReferenceAmb(const AmbKey obs_key, double* val) {
  *val = 0.0;
  bool b_exist = false;
  AmbKey ref;
  for (unsigned int i = 0; i < reference_sat_.size(); ++i) {
    ref = reference_sat_[i];
    if (ref.band_id == obs_key.band_id) {
      b_exist = true;
      break;
    }
  }
  if (!b_exist) {
    *val = 0.0;
    return false;
  }
  if (obs_key == ref) {
    *val = 0.0;
    return true;
  }
  b_exist = GetLog(ref.band_id, ref.sat_prn, amb_fixed_, val);
  return b_exist;
}

// deploy _amb_track to generate the transform matrix
Eigen::MatrixXd AmbiguityTracker::GetMatrixSD2DD(Eigen::MatrixXd* ref_amb) {
  unsigned int max_amb_num = unresolved_phase_.size();
  int dd_obs_num = 0;
  Eigen::MatrixXd matrix_sd_2_dd(max_amb_num, max_amb_num);
  Eigen::MatrixXd ref_sd_amb(max_amb_num, 1);
  matrix_sd_2_dd.setZero();
  ref_sd_amb.setZero();
  dd_obs_num = 0;
  for (unsigned int i = 0; i < max_amb_num; ++i) {
    int ref_ind = GetReferenceIndex(unresolved_phase_[i]);
    if (ref_ind == -1) {
      // it is reference obs
      continue;
    }
    if (ref_ind >= 0) {
      matrix_sd_2_dd(dd_obs_num, ref_ind) = -1;
    } else {
      // ref_ind == -2,indicating reference obs' amb fixed
      double val = 0.0;
      GetReferenceAmb(unresolved_phase_[i], &val);
      ref_sd_amb(dd_obs_num, 0) = val;
    }
    matrix_sd_2_dd(dd_obs_num, i) = 1;
    ++dd_obs_num;
  }
  *ref_amb = ref_sd_amb.topLeftCorner(dd_obs_num, 1);
  return matrix_sd_2_dd.topLeftCorner(dd_obs_num, max_amb_num);
}

bool AmbiguityTracker::UpdateSDAmb(const Eigen::MatrixXd &float_sd,
                                     Eigen::MatrixXd* integer_sd) {
  unsigned int size_int = integer_sd->rows();
  for (unsigned int i = 0; i < size_int; ++i) {
    AmbKey temp = unresolved_phase_[i];
    int ref_ind = GetReferenceIndex(temp);
    if (ref_ind >= 0) {
      (*integer_sd)(i, 0) += (float_sd(ref_ind, 0));
    } else if (ref_ind == -1) {
      (*integer_sd)(i, 0) = (float_sd(i, 0));
    } else if (ref_ind == -2) {
      // related reference phase ambiguity is fixed,no update to integer_sd
      // do nothing
    }
  }
  return true;
}

bool AmbiguityTracker::AddFixedAmb(const Eigen::MatrixXd &integer_sd) {
  unsigned int size_int = integer_sd.rows();
  for (unsigned int i = 0; i < size_int; ++i) {
    AmbKey temp = unresolved_phase_[i];
    double val = integer_sd(i, 0);
    SetLog(temp.band_id, temp.sat_prn, val, &amb_fixed_);
  }
  // erase unfixed log
  unresolved_phase_.resize(0);
  return true;
}

bool AmbiguityTracker::DeleteDescendingAmb() {
  std::vector<apollo::drivers::gnss::GnssBandID> del_ref_band;
  del_ref_band.resize(0);

  std::map<AmbKey, double>::iterator iter_amb = amb_fixed_.begin();
  for (iter_amb = amb_fixed_.begin(); iter_amb != amb_fixed_.end();) {
    AmbKey temp = iter_amb->first;
    std::map<AmbKey, double>::iterator iter_elv = obs_elevation_deg_.find(temp);
    if (iter_elv == obs_elevation_deg_.end()) {
      // take care of the erased satellite being a reference satellite
      for (unsigned int ind = 0; ind < reference_sat_.size(); ++ind) {
        if (temp == reference_sat_[ind]) {
          reference_sat_.erase(reference_sat_.begin() + ind);
          del_ref_band.push_back(temp.band_id);
          break;
        }
      }
      // erase fixed amb not in current observation
      amb_fixed_.erase(iter_amb++);
    } else {
      ++iter_amb;
    }
  }
  // reset certain band' reference satellite
  for (unsigned int ind = 0; ind < del_ref_band.size(); ++ind) {
    apollo::drivers::gnss::GnssBandID band_id = del_ref_band[ind];
    double max_elv_deg = -0.1;
    AmbKey reference;
    for (iter_amb = amb_fixed_.begin(); iter_amb != amb_fixed_.end();
         ++iter_amb) {
      AmbKey temp = iter_amb->first;
      if (temp.band_id != band_id) {
        continue;
      }
      std::map<AmbKey, double>::iterator iter_elv =
          obs_elevation_deg_.find(temp);
      if (iter_elv != obs_elevation_deg_.end()) {
        if (iter_elv->second > max_elv_deg) {
          max_elv_deg = iter_elv->second;
          reference = temp;
        }
      } else {
        continue;
      }
    }
    if (max_elv_deg > 0.0) {
      reference_sat_.push_back(reference);
    }
  }
  return true;
}
bool AmbiguityTracker::DeleteSlipAmb(
    const apollo::drivers::gnss::GnssBandID &sat_band_id,
    const unsigned int sat_prn) {
  AmbKey slip_obs(sat_band_id, sat_prn);
  bool b_delete_ref = false;
  // erase fixed amb for slips
  DeleteLog(sat_band_id, sat_prn, &amb_fixed_);
  // take care of the sliped satellite being a reference satellite
  for (unsigned int ind = 0; ind < reference_sat_.size(); ++ind) {
    if (slip_obs == reference_sat_[ind]) {
      reference_sat_.erase(reference_sat_.begin() + ind);
      b_delete_ref = true;
      break;
    }
  }
  if (!b_delete_ref) {
    return true;
  }

  return true;
}

bool AmbiguityTracker::ReupdateReferenceAmb(
    const double sd_amb_res[apollo::drivers::gnss::GLO_G3]) {
  // amb_fixed_
  std::map<AmbKey, double>::iterator iter = amb_fixed_.begin();
  for (; iter != amb_fixed_.end(); ++iter) {
    iter->second += sd_amb_res[iter->first.band_id];
  }
  return true;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
