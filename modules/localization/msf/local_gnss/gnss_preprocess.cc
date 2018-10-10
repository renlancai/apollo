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

#include "modules/localization/msf/local_gnss/gnss_preprocess.h"
#include "modules/localization/msf/local_gnss/atmosphere.h"
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"
#include "modules/localization/msf/local_gnss/pvt_satellite.h"
namespace apollo {
namespace localization {
namespace local_gnss {

PreProcessor::PreProcessor() { Initialize(); }

PreProcessor::~PreProcessor() { global_ephemeris_ptr_ = NULL; }

void PreProcessor::Initialize() {
  debug_print_ = false;
  user_id_ = 0;
  // 2mm
  const double unit_weight = 0.002 * 0.002;
  phase_precision_ = sqrt(2.0) * unit_weight;
  global_ephemeris_ptr_ = NULL;
  band_to_solve_.resize(0);
  gnss_to_solve_.resize(0);
  last_sat_list_.resize(0);
  last_obs_time_ = -0.1;
  slip_recorder_.clear();

  last_pos_std_.x = 9.9 * 9.9;
  last_pos_std_.y = 9.9 * 9.9;
  last_pos_std_.z = 9.9 * 9.9;
  precise_set_coor_ = false;
  delta_time_ = 0.0;
}
bool PreProcessor::ReInitialize() {
  last_sat_list_.resize(0);
  last_obs_time_ = -0.1;
  last_pos_std_.x = 9.9 * 9.9;
  last_pos_std_.y = 9.9 * 9.9;
  last_pos_std_.z = 9.9 * 9.9;
  precise_set_coor_ = false;
  delta_time_ = 0.0;
  return true;
}
void PreProcessor::Reset() { slip_recorder_.clear(); }

bool PreProcessor::CheckSlipValue(const GnssBandID& band_id,
                              const unsigned int sat_prn, double* slip_val) {
  SlipKey temp(band_id, sat_prn);
  std::map<SlipKey, double>::iterator iter = slip_recorder_.find(temp);
  if (iter != slip_recorder_.end()) {
    *slip_val = iter->second;
    return true;
  }
  return false;
}

void PreProcessor::SetGlobalEphemerisPtr(SatelliteInterface* ptr_eph) {
  global_ephemeris_ptr_ = reinterpret_cast<SatelliteInterface*>(ptr_eph);
  // global_ephemeris_ptr_ = (SatelliteInterface*)ptr_eph;
}

unsigned int PreProcessor::GetSlipSize() { return slip_recorder_.size(); }

PointThreeDim PreProcessor::GetVelocity() { return velocity_enu_; }

PointThreeDim PreProcessor::GetVelocityStd() {
  return std_vel_.SquareRoot();
}

PointThreeDim PreProcessor::GetDeltaPos() { return delta_pos_; }

PointThreeDim PreProcessor::GetPositionStd() {
  PointThreeDim temp(0.0001, 0.0001, 0.0001);
  temp = last_pos_std_ + temp * delta_time_;
  return temp.SquareRoot();
}

void PreProcessor::SetPresiceCoor(const double upt_time,
                               const PointThreeDim& rtk_result,
                               const PointThreeDim& std_result) {
  precise_set_coor_ = true;
  last_pos_ = rtk_result;
  last_pos_std_ = std_result;
  delta_time_ = 0.0;
}

bool PreProcessor::IsPreciseSet() { return precise_set_coor_; }

void PreProcessor::EnablePrint(const bool& d_print) { debug_print_ = d_print; }

void PreProcessor::PrintMatrix(const Eigen::MatrixXd& t, const char* t_name) {
  if (!debug_print_) {
    return;
  }
  gnss_utility::PrintEigenMatrix(t, t_name);
}

bool PreProcessor::CalculateAverageVel(
    const PointThreeDim& pos,
    const EpochObservation& newobs,
    PointThreeDim* pos_corr) {
  PointThreeDim dxyz = pos;
  PointThreeDim denu;
  dxyz = dxyz + *pos_corr - last_pos_;
  delta_pos_ = dxyz;
  // pos += *pos_corr;
  gnss_utility::dxyz2enu(pos, dxyz, &denu);
  double t_gap =
      SECOND_PER_WEEK * (newobs.gnss_week() - last_obs_.gnss_week()) +
      newobs.gnss_second_s() - last_obs_.gnss_second_s();
  velocity_enu_ = denu / t_gap;
  return true;
}

int PreProcessor::CheckBandPhaseObs(const BandObservation& band_obs1,
                                       const BandObservation& band_obs2) {
  if (band_obs1.band_id() != band_obs2.band_id()) {
    return -1;
  }
  if (gnss_utility::GetIndexInVector(band_obs1.band_id(), band_to_solve_) ==
      -1) {
    return -1;
  }
  if (fabs(band_obs1.carrier_phase()) <= 0.0) {
    return -1;
  }
  if (fabs(band_obs2.carrier_phase()) <= 0.0) {
    return -1;
  }
  return 0;
}

bool PreProcessor::SetResolvedBands(
    const std::vector<apollo::drivers::gnss::GnssBandID>& bands) {
  band_to_solve_.resize(0);
  unsigned int size = bands.size();
  for (unsigned int i = 0; i < size; ++i) {
    band_to_solve_.push_back(bands[i]);
  }
  unsigned int size_band = band_to_solve_.size();
  if (size_band <= 0) {
    return false;
  }
  // generate gnss_sys_vector based on gnss_band vector
  gnss_to_solve_.resize(0);
  for (unsigned int i = 0; i < size_band; ++i) {
    GnssType temp = global_ephemeris_ptr_->DetermineGnssType(band_to_solve_[i]);
    if (temp == apollo::drivers::gnss::SYS_UNKNOWN) {
      continue;
    }
    if (gnss_utility::GetIndexInVector(temp, gnss_to_solve_) == -1) {
      gnss_to_solve_.push_back(temp);
    }
  }
  return gnss_to_solve_.size() > 0;
}

#if 0
bool PreProcessor::fix_observation(
        const SlipKey& band_obs_id,
        const double slip,
        EpochObservation& newobs) {
    const apollo::drivers::gnss::GnssBandID band_id = band_obs_id.band_id;
    GnssType temp = apollo::drivers::gnss::GPS_SYS;
    if (band_id == apollo::drivers::gnss::GPS_L1 ||
        band_id == apollo::drivers::gnss::GPS_L2 ||
        band_id == apollo::drivers::gnss::GPS_L5) {
        temp = apollo::drivers::gnss::GPS_SYS;
    } else if (band_id == apollo::drivers::gnss::BDS_B1 ||
        band_id == apollo::drivers::gnss::BDS_B2 ||
        band_id == apollo::drivers::gnss::BDS_B3) {
        temp = apollo::drivers::gnss::BDS_SYS;
    } else if (band_id == apollo::drivers::gnss::GLO_G1 ||
        band_id == apollo::drivers::gnss::GLO_G2) {
        temp = apollo::drivers::gnss::GLO_SYS;
    }

    unsigned int obs_num = newobs.sat_obs_num();
    for (unsigned int i = 0; i < obs_num; ++i) {
        SatelliteObservation sat_i = newobs.sat_obs(i);
        if (sat_i.sat_sys() != temp || sat_i.sat_prn() != band_obs_id.sat_prn) {
            continue;
        }
        for (unsigned int j = 0; j < sat_i.band_obs_num(); ++j) {
            if (sat_i.band_obs(j).band_id() == band_id) {
                double phase = sat_i.band_obs(j).carrier_phase();
                phase -= slip;
                sat_i.mutable_band_obs(j)->set_carrier_phase(phase);
                *(newobs.mutable_sat_obs(i)) = sat_i;
                return true;
            }
        }
    }
    return false;
}

bool PreProcessor::update_last_epoch_satellite(
        unsigned int gnss_week_num,
        double gnss_second_s) {
    if (global_ephemeris_ptr_ != NULL) {
        return false;
    }
    unsigned int last_second_s = (unsigned int)last_obs_.gnss_second_s();
    const unsigned int glonass_epoch = 1800;
    const unsigned int bds_epoch = 3600;
    const unsigned int gps_epoch = 7200;
    if ((last_second_s - 15 * 60 - 15) % glonass_epoch == 0 ||
        (last_second_s - 14) % bds_epoch == 0 ||
        last_second_s % gps_epoch == 0) {
        return true;
    }
    return false;
}
#endif

bool PreProcessor::UpdateSatInforWithNewEph(SatelliteInfor* last_sat) {
  if (global_ephemeris_ptr_ == NULL) {
    return false;
  }
  return global_ephemeris_ptr_->GetPosVelClock(
      last_sat->sat_sys, last_sat->sat_prn, last_sat->gnss_week,
      last_sat->time_signal_transmitted, last_sat->time_travles,
      &last_sat->position, &last_sat->velocity, &last_sat->clock_bias,
      &last_sat->clock_drift, &last_sat->toe);
}

bool PreProcessor::SaveLastEpochInfor(
    const PointThreeDim& pos,
    const EpochObservation& newobs,
    const std::vector<SatelliteInfor>& sat_list) {
  last_pos_ = pos;
  last_obs_ = newobs;
  last_sat_list_.resize(0);
  unsigned int size_current = sat_list.size();
  for (unsigned i = 0; i < size_current; i++) {
    last_sat_list_.push_back(sat_list[i]);
  }
  return true;
}

bool PreProcessor::DetectBasedOnGeometry(
    const std::vector<GnssType>& related_gnss_type,
    const PointThreeDim& pos,
    const EpochObservation& newobs,
    const std::vector<SatelliteInfor>& sat_list) {
  if (global_ephemeris_ptr_ == NULL) {
    return false;
  }
  // clear last epoch slip-detecting result
  Reset();

  Atmosphere trop_current(pos, newobs.gnss_week(), newobs.gnss_second_s());
  Atmosphere trop_last(last_pos_, last_obs_.gnss_week(),
                       last_obs_.gnss_second_s());
  const double sec_per_week = apollo::localization::local_gnss::SECOND_PER_WEEK;
  double new_obs_time =
      newobs.gnss_week() * sec_per_week + newobs.gnss_second_s();

  // support 10 Hz at most
  if (fabs(last_obs_time_ - new_obs_time) < 0.1) {
    last_obs_time_ = new_obs_time;
    SaveLastEpochInfor(pos, newobs, sat_list);
    return false;
  }
  if (fabs(last_obs_time_ - new_obs_time) > 5.0 &&
      fabs(last_obs_time_) > 10.0) {
    last_obs_time_ = new_obs_time;
    SaveLastEpochInfor(pos, newobs, sat_list);
    last_pos_std_.x = 9.9 * 9.9;
    last_pos_std_.y = 9.9 * 9.9;
    last_pos_std_.z = 9.9 * 9.9;
    precise_set_coor_ = false;
    return false;
  }
  delta_time_ += fabs(new_obs_time - last_obs_time_);
  last_obs_time_ = new_obs_time;

  const unsigned int gnss_sys_num = related_gnss_type.size();
  const unsigned int max_sat_num = sat_list.size();
  // at most triple frequencies and virtual x/y/z observations
  const unsigned int max_common_obs_num = max_sat_num * 3 + 3;
  // relative clock offsets + diff_x/y/z
  unsigned int num_states = 3 + gnss_sys_num;

  // carrier_phase
  Eigen::MatrixXd al(max_common_obs_num, num_states);
  Eigen::MatrixXd vl(max_common_obs_num, 1);
  Eigen::MatrixXd pl(max_common_obs_num, max_common_obs_num);
  // initilization
  al.setZero();
  vl.setZero();
  pl.setIdentity(max_common_obs_num, max_common_obs_num);

  double deg2rad = apollo::localization::local_gnss::PI / 180.0;
  // store the wave length and obs_key
  std::vector<double> len_recorder;
  len_recorder.resize(0);
  std::vector<SlipKey> obs_recorder;
  obs_recorder.resize(0);

  unsigned int common_phase_num = 0;
  unsigned int common_sat_num = 0;
  // begin current epoch detecting
  unsigned int size_last = last_sat_list_.size();
  unsigned int size_current = sat_list.size();

  std::vector<GnssType> common_related_gnss_type;
  common_related_gnss_type.resize(0);
  common_related_gnss_type.resize(0);
  for (unsigned int i = 0; i < size_last; ++i) {
    for (unsigned int j = 0; j < size_current; ++j) {
      if (!last_sat_list_[i].IsSameSatellite(sat_list[j])) {
        continue;
      }
      GnssType type = sat_list[j].sat_sys;
      if (gnss_utility::GetIndexInVector(type, gnss_to_solve_) == -1) {
        continue;
      }
      bool sat_valid = false;
      // for external Reset of last_pos_ by RTK
      last_sat_list_[i].distance =
          gnss_utility::GetDistance(last_pos_, last_sat_list_[i].position);
      if (last_sat_list_[i].toe != sat_list[j].toe) {
        // update last sat pos deploying new eph
        last_sat_list_[i].toe = sat_list[j].toe;
        if (!UpdateSatInforWithNewEph(&last_sat_list_[i])) {
          continue;
        }
        if (last_sat_list_[i].toe != sat_list[j].toe) {
          continue;
        }
        // update new distance
        last_sat_list_[i].distance =
            gnss_utility::GetDistance(last_pos_, last_sat_list_[i].position);
      }
      // common satellite
      double last_trop_delay =
          trop_last.TropoDelay(last_sat_list_[i].elevation * deg2rad,
                                last_sat_list_[i].azimuth * deg2rad);
      double current_trop_delay = trop_current.TropoDelay(
          sat_list[j].elevation * deg2rad, sat_list[j].azimuth * deg2rad);
      // get common sat
      SatelliteObservation last_sat_j;
      SatelliteObservation current_sat_j;
      assert(last_sat_list_[i].index_in_obs >= 0 &&
             sat_list[j].index_in_obs >= 0);
      last_sat_j = last_obs_.sat_obs(last_sat_list_[i].index_in_obs);
      current_sat_j = newobs.sat_obs(sat_list[j].index_in_obs);

      // every band of GNSS in pre-defined settings construct the equation
      for (unsigned int m = 0; m < current_sat_j.band_obs_num(); ++m) {
        for (unsigned int n = 0; n < last_sat_j.band_obs_num(); ++n) {
          if (CheckBandPhaseObs(current_sat_j.band_obs(m),
                                   last_sat_j.band_obs(n)) < 0) {
            continue;
          }
          int glo_fre_num = 0;
          // need a real-time update from interface
          double len = SatelliteInterface::GetBandLength(
              sat_list[j].sat_prn, current_sat_j.band_obs(m).band_id(),
              &glo_fre_num);
          // phase  process
          al(common_phase_num, 0) = sat_list[j].direction.x;
          al(common_phase_num, 1) = sat_list[j].direction.y;
          al(common_phase_num, 2) = sat_list[j].direction.z;
          vl(common_phase_num, 0) =
              current_sat_j.band_obs(m).carrier_phase() * len -
              last_sat_j.band_obs(n).carrier_phase() * len -
              (sat_list[j].distance - last_sat_list_[i].distance);
          // ztd update
          vl(common_phase_num, 0) -= (current_trop_delay - last_trop_delay);
          // index of clock error coefficient
          int sat_index_in_sys =
              gnss_utility::GetIndexInVector(type, common_related_gnss_type);
          if (sat_index_in_sys == -1) {
            common_related_gnss_type.push_back(type);
            sat_index_in_sys = common_related_gnss_type.size() - 1;
          }
          // clock
          al(common_phase_num, 3 + sat_index_in_sys) = 1;
          // pl(common_phase_num, common_phase_num) = sina / phase_precision_;
          ++common_phase_num;
          len_recorder.push_back(len);
          SlipKey temp(current_sat_j.band_obs(m).band_id(),
                       sat_list[j].sat_prn);
          obs_recorder.push_back(temp);
          sat_valid = true;
          break;
        }
      }
      if (sat_valid == true) {
        ++common_sat_num;
      }
      break;
    }
  }

  if (common_phase_num <= 0 ||
      common_sat_num < 4 + common_related_gnss_type.size()) {
    SaveLastEpochInfor(pos, newobs, sat_list);
    precise_set_coor_ = false;
    return false;
  }
  // To do: add dx,dy,dz constrains to only esitmate relative clock drift
  num_states = 3 + common_related_gnss_type.size();
  Eigen::MatrixXd dx(num_states, 1);
  dx.setZero();
  Eigen::MatrixXd an = al.topLeftCorner(common_phase_num, num_states);
  Eigen::MatrixXd ln = vl.topRows(common_phase_num);
  Eigen::MatrixXd pn = pl.topLeftCorner(common_phase_num, common_phase_num);
  Eigen::MatrixXd atpa = an.transpose() * pn * an;
  Eigen::MatrixXd inv_atpa = atpa.inverse();
  Eigen::MatrixXd atpl = an.transpose() * pn * ln;
  dx = inv_atpa * atpl;

  PrintMatrix(an, "an");
  PrintMatrix(ln, "ln");
  PrintMatrix(dx, "dx");
  Eigen::MatrixXd resi = ln - an * dx;
  PrintMatrix(resi, "resi");
  Eigen::MatrixXd sum = resi.transpose() * resi;

  // position dop of XYZ direction
  Eigen::MatrixXd dop = an.transpose() * an;
  dop = dop.inverse();
  PointThreeDim slip_dop(dop(0, 0), dop(1, 1), dop(2, 2));

  // velocity dop of ENU direction
  Eigen::Matrix3d matrix_r = gnss_utility::DcmEcef2Navi(pos);
  Eigen::Matrix3d dop_enu =
      matrix_r * dop.topLeftCorner(3, 3) * matrix_r.transpose();
  PointThreeDim v_enu_dop(dop_enu(0, 0), dop_enu(1, 1), dop_enu(2, 2));

  double sum_res = sum(0, 0);
  double std_v = sqrt(sum_res / (resi.rows() - 3));
  double std_threshold = 0.02;
  const double precision_sd_phase = 0.02;
  if (std_v < std_threshold) {
    PointThreeDim dx_corr(dx(0, 0), dx(1, 0), dx(2, 0));
    CalculateAverageVel(pos, newobs, &dx_corr);

    PointThreeDim pos_corrected = pos;
    pos_corrected = pos_corrected + dx_corr;
    // no slip happens
    SaveLastEpochInfor(pos_corrected, newobs, sat_list);
    std_vel_ = v_enu_dop * precision_sd_phase * precision_sd_phase;
    last_pos_std_ += slip_dop * precision_sd_phase * precision_sd_phase;
    return true;
  }
  // identify slips
  unsigned int num_itera = 0;
  Eigen::MatrixXd px(pn.rows(), pn.cols());
  px.setIdentity();

  unsigned int max_num_itera = 5;
  double average_resi = 0.0;
  for (unsigned int i = 0; i < common_phase_num; ++i) {
    average_resi += fabs(resi(i, 0));
  }
  average_resi /= common_phase_num;

  while (true) {
    ++num_itera;
    double sum_resi = 0.0;
    for (unsigned int i = 0; i < common_phase_num; ++i) {
      //
      if (fabs(resi(i, 0)) > 4 * std_v) {
        px(i, i) *= 0.01;
      } else if (fabs(resi(i, 0)) > 3 * std_v) {
        px(i, i) *= 0.1;
      } else if (fabs(resi(i, 0)) > 2 * std_v) {
        px(i, i) *= 0.5;
      }
      //
      if (fabs(resi(i, 0)) > 4 * average_resi) {
        px(i, i) *= 0.01;
      } else if (fabs(resi(i, 0)) > 3 * average_resi) {
        px(i, i) *= 0.1;
      } else if (fabs(resi(i, 0)) > 2 * average_resi) {
        px(i, i) *= 0.5;
      }
      sum_resi += fabs(resi(i, 0));
    }
    average_resi = sum_resi / common_phase_num;
    atpa = an.transpose() * px * an;
    inv_atpa = atpa.inverse();
    atpl = an.transpose() * px * ln;
    dx = inv_atpa * atpl;
    resi = ln - an * dx;
    PrintMatrix(dx, "dx");
    PrintMatrix(resi, "resi");
    sum = resi.transpose() * resi;
    Eigen::MatrixXd test = resi.transpose() * px * resi;
    double sum_test = test(0, 0);
    sum_test = sqrt(sum_test / (resi.rows()));
    sum_res = sum(0, 0);
    std_v = sqrt(sum_res / (resi.rows()));
    if (std_v < std_threshold || num_itera >= max_num_itera) {
      break;
    }
  }

  std_vel_ = v_enu_dop * precision_sd_phase * precision_sd_phase;
  if (std_v < std_threshold) {
    last_pos_std_ += slip_dop * precision_sd_phase * precision_sd_phase;
  } else {
    last_pos_std_.x = 0.1 * 0.1;
    last_pos_std_.y = 0.2 * 0.2;
    last_pos_std_.z = 0.1 * 0.1;
  }

  // roughly evaluate
  unsigned int num_slips = 0;
  for (unsigned int i = 0; i < common_phase_num; ++i) {
    if (fabs(resi(i, 0)) > 3 * std_v || fabs(resi(i, 0)) > 0.05) {
      double val = resi(i, 0) / len_recorder[i];
      if (fabs(val) > 0.1) {
        SlipKey temp = obs_recorder[i];
        // specific actions should be executed in in main gnss rtk solution.
        slip_recorder_.insert(std::map<SlipKey, double>::value_type(temp, val));
        ++num_slips;
      }
    }
  }
  PointThreeDim dx_corr(dx(0, 0), dx(1, 0), dx(2, 0));
  CalculateAverageVel(pos, newobs, &dx_corr);
  PointThreeDim pos_corrected = pos;
  pos_corrected = pos_corrected + dx_corr;
  SaveLastEpochInfor(pos_corrected, newobs, sat_list);
  return true;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
