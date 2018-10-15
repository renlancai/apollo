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
#include <vector>
#include "modules/localization/msf/local_gnss/atmosphere.h"
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"

namespace apollo {
namespace localization {
namespace local_gnss {

// setting interface for initialize
bool GnssPntSolver::SetPositionOption(const PositionOption position_option) {
  position_option_ = position_option;
  return true;
}

PositionOption GnssPntSolver::GetPositionOption() { return position_option_; }

bool GnssPntSolver::SetTropsphereOption(const TropsphereOption trop_option) {
  ztd_option_ = trop_option;
  return true;
}

bool GnssPntSolver::SetIonosphereOption(const IonosphereOption iono_option) {
  iono_option_ = iono_option;
  return true;
}

bool GnssPntSolver::SetElevationCutoffDeg(const double elevation_cutoff) {
  elevation_cutoff_deg_ = elevation_cutoff;
  return true;
}

double GnssPntSolver::GetElevationCutoffDeg() {
  return elevation_cutoff_deg_;
}

void GnssPntSolver::SetBaserBufferSize(const unsigned int baser_buffer_size) {
  max_size_baser_buffer_ = baser_buffer_size;
}

int GnssPntSolver::GetBaserBufferSize() { return max_size_baser_buffer_; }

void GnssPntSolver::SetMaxGpsEphSize(const unsigned int max_gps_eph_size) {
  global_ephemeris_.SetEphMaxsize(max_gps_eph_size,
                                     apollo::drivers::gnss::GPS_SYS);
}

void GnssPntSolver::SetMaxBdsEphSize(const unsigned int max_bds_eph_size) {
  global_ephemeris_.SetEphMaxsize(max_bds_eph_size,
                                     apollo::drivers::gnss::BDS_SYS);
}
void GnssPntSolver::SetMaxGloEphSize(const unsigned int max_glo_eph_size) {
  global_ephemeris_.SetEphMaxsize(max_glo_eph_size,
                                     apollo::drivers::gnss::GLO_SYS);
}

void GnssPntSolver::SetMaxGnssSystem(const unsigned int max_gnss_sys_num) {
  max_gnss_sys_num_ = max_gnss_sys_num;
}

void GnssPntSolver::EnableHalfCycleAr(const bool b_enable) {
  enable_half_cycle_ar_ = b_enable;
}

bool GnssPntSolver::GetHalfCycleAr() { return enable_half_cycle_ar_; }

void GnssPntSolver::EnanleCycleSlipFix() { enable_cycle_slip_fix_ = true; }

bool GnssPntSolver::GetCycleSlipFix() { return enable_cycle_slip_fix_; }

void GnssPntSolver::SetGlonassIfb(double ifb) { glonass_ifb_ = ifb; }

double GnssPntSolver::GetGlonassIfb() { return glonass_ifb_; }

void GnssPntSolver::SetBaserCoordinate(const PointThreeDim& baser_coor) {
  baser_coor_ = baser_coor;
  external_fix_baser_ = true;
}

void GnssPntSolver::SetBaserCoordinate(double x, double y, double z) {
  SetBaserCoordinate(PointThreeDim(x, y, z));
}

PointThreeDim GnssPntSolver::GetBaserCoor() { return baser_coor_; }

void GnssPntSolver::SetEnableExternalPrediction(bool b_enable) {
  enable_external_prediction_ = b_enable;
}

bool GnssPntSolver::GetEnableExternalPrediction() {
  return enable_external_prediction_;
}

void GnssPntSolver::SetEnableTdPhaseUpdate(bool b_enable) {
  enable_td_phase_update_ = b_enable;
}

bool GnssPntSolver::GetEnableTdPhaseUpdate() {
  return enable_td_phase_update_;
}

void GnssPntSolver::SetDebugPrint(const bool& d_print) {
  debug_print_ = d_print;
  rover_preprocessor_.EnablePrint(d_print);
}

void GnssPntSolver::SetSynchTimeGapThreshold(
    const double& synch_time_gap_threshold) {
  synch_time_gap_threshold_ = synch_time_gap_threshold;
}

double GnssPntSolver::GetSynchTimeGapThreshold() {
  return synch_time_gap_threshold_;
}

bool GnssPntSolver::SetRtkResultFile(char* rtk_result_file) {
  if (rtk_result_file != NULL) {
    file_rtk_result_ = rtk_result_file;
    fp_rtk_result_ = fopen(rtk_result_file, "w+");
    return true;
  }
  return false;
}

bool GnssPntSolver::SetAmbiguityFile(char* ambiguity_recorder_file) {
  if (ambiguity_recorder_file != NULL) {
    file_amb_log_ = ambiguity_recorder_file;
    return true;
  }
  return false;
}

bool GnssPntSolver::SaveGnssEphemris(
    const GnssEphemeris& gnss_orbit) {
  if (!(gnss_orbit.gnss_type() == apollo::drivers::gnss::GPS_SYS ||
        gnss_orbit.gnss_type() == apollo::drivers::gnss::BDS_SYS ||
        gnss_orbit.gnss_type() == apollo::drivers::gnss::GLO_SYS)) {
    return false;
  }
  return global_ephemeris_.SaveEphemeris(gnss_orbit);
}

bool GnssPntSolver::SaveBaserObservation(const EpochObservation& baser_obs) {
  // To do: distinguish different baser obs with their receiver_id to support
  // multi-baser mode.
  if (!(baser_obs.has_position_x() && baser_obs.has_position_y() &&
        baser_obs.has_position_z())) {
    EncodeDetails("invalid baser obs without coordinate binded.");
    return false;
  }
  PointThreeDim coor_decoded(baser_obs.position_x(), baser_obs.position_y(),
                             baser_obs.position_z());
  // remove abnormal RTCM decoder with loss of lots satellite observations.
  // supposing the base satellites number is stable.
  if (baser_obs.sat_obs_num() <= max_sat_num_baser_ / 2) {
    return false;
  }
  PointThreeDim temp_baser = baser_coor_;
  std::vector<SatelliteInfor> sate_vector_used;
  std::vector<GnssType> related_gnss_type;
  PointThreeDim spp_dop;
  double std_baser = GetStandardPosition(baser_obs, &sate_vector_used,
                                       &related_gnss_type, &temp_baser,
                                       &spp_dop);

  double dis_gap = gnss_utility::GetDistance(temp_baser, coor_decoded);
  if (!CheckBaserObs(std_baser, dis_gap)) {
    EncodeDetails(
        "invalid baser obs whose coordinate binded is wrong with SPP.");
    return false;
  }
  if (baser_obs.sat_obs_num() > max_sat_num_baser_) {
    max_sat_num_baser_ = baser_obs.sat_obs_num();
  }
  baser_obs_.push_back(baser_obs);
  if (baser_obs_.size() <= max_size_baser_buffer_) {
    return true;
  }
  // erase the first arriving over-due epoch data
  baser_obs_.erase(baser_obs_.begin());
  return true;
}

bool GnssPntSolver::GetFixStatus() { return b_rtkfixed_; }

void GnssPntSolver::ClearFixedRtk() { ResetFixedRtk(); }

double GnssPntSolver::GetLeapSecond(unsigned int gps_week_num,
                                      double gps_week_second_s) {
  return global_ephemeris_.GetCurrentLeapSecond(gps_week_num,
                                                gps_week_second_s);
}

double GnssPntSolver::GetRatio() { return ratio_; }

std::string GnssPntSolver::GetInvalidRtkDetail() {
  std::string temp = invalid_rtk_sub_infor_;
  return temp;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
