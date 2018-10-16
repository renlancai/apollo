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

// SEMI_CYCYLE_PHASE processing is not supported yet.
#define _NO_SEMI_CYCLE_PHASE_

GnssPntSolver::GnssPntSolver() { Initialize(); }

GnssPntSolver::~GnssPntSolver() {
  if (fp_rtk_result_ != NULL) {
    fclose(fp_rtk_result_);
  }
}

void GnssPntSolver::Initialize() {
  range_precision_ = 0.0;
  num_valid_sat_ = 0;
  b_rtkfixed_ = false;
  position_option_ = STANDARD;
  ztd_option_ = ZTD_CORRECT;
  iono_option_ = IONO_OFF;
  elevation_cutoff_deg_ = 15.0;  // deg
  ratio_ = 0.0;
  ratio_epoch_num_ = 0;
  _std_v = 0.0;
  gnss_weeknum_ = 0;
  gnss_week_second_s_ = -0.1;
  max_gnss_sys_num_ = 3;

  max_size_rover_buffer_ = 0;

  // default solver containing all kinds of frequencies
  band_to_solve_.resize(0);
  band_to_solve_.push_back(apollo::drivers::gnss::GPS_L1);
  band_to_solve_.push_back(apollo::drivers::gnss::GPS_L2);
  // not evaluated yet
  // band_to_solve_.push_back(apollo::drivers::gnss::GPS_L5);

  band_to_solve_.push_back(apollo::drivers::gnss::BDS_B1);
  band_to_solve_.push_back(apollo::drivers::gnss::BDS_B2);
  // not evaluated yet
  // band_to_solve_.push_back(apollo::drivers::gnss::BDS_B3);

  band_to_solve_.push_back(apollo::drivers::gnss::GLO_G1);
  band_to_solve_.push_back(apollo::drivers::gnss::GLO_G2);
  // not evaluated yet
  // band_to_solve_.push_back(apollo::drivers::gnss::GLO_G3);

  GetGnssSystemToSolve();
  ClearSequential();
  // set pre-processor option
  enable_half_cycle_ar_ = false;
  enable_cycle_slip_fix_ = false;
  rover_preprocessor_.SetResolvedBands(band_to_solve_);
  baser_preprocessor_.SetResolvedBands(band_to_solve_);
  rover_preprocessor_.SetGlobalEphemerisPtr(&global_ephemeris_);
  baser_preprocessor_.SetGlobalEphemerisPtr(&global_ephemeris_);

  const double unit_weight = 1.0 * 1.0;
  //  sqrt(2) for single difference
  double phase_p = 2 * 0.01 * 0.01;
  phase_precision_ = phase_p / unit_weight;

  glonass_ifb_ = 0.0;
  external_fix_baser_ = false;
  enable_external_prediction_ = false;
  enable_td_phase_update_ = false;
  max_size_baser_buffer_ = 10;

  // set default buffer size for real time ephemris
  unsigned int num_day = 2;
  const int num_per_day_gps = 12;  // one epoch two hours
  const int num_per_day_bds = 24;  // one epoch one hour
  const int num_per_day_glo = 96;  // one epoch 15 minutes
  SetMaxGpsEphSize(num_day * num_per_day_gps);
  SetMaxBdsEphSize(num_day * num_per_day_bds);
  SetMaxGloEphSize(num_day * num_per_day_glo);

  // debug flag
  debug_print_ = false;
  counter_for_low_ratio_ = 0;
  num_sd_phase_recursive_ = 0;

  file_rtk_result_ = NULL;
  fp_rtk_result_ = NULL;
  file_amb_log_ = NULL;

  _predict_time_sec = -999.9;
  test_ins_aid_ = false;

  tough_fixing_pool_.clear();
  max_sat_num_baser_ = 0;
  real_time_ub_fail_rate_ = 0;

  invalid_rtk_sub_infor_ = "constructor";
  distance_rover2baser_ = 0.0;
}

bool GnssPntSolver::SynchronizeBaser(
    const unsigned int gnss_week_num, const double gnss_week_second_s,
    EpochObservation* baser_obs,
    const double& gap_threshold) {
  unsigned int size = baser_obs_.size();
  if (size <= 0) {
    return false;
  }
  // guarantee valid baser output
  *baser_obs = baser_obs_[size - 1];
  double sec_per_week = apollo::localization::local_gnss::SECOND_PER_WEEK;
  double t_rover = gnss_week_num * sec_per_week + gnss_week_second_s;
  double t_baser =
      baser_obs->gnss_week() * sec_per_week + baser_obs->gnss_second_s();
  if (t_rover > t_baser) {
    return true;
  }
  if (fabs(t_baser - t_rover) < gap_threshold) {
    return true;
  }
  double tgap = t_rover - t_baser;
  for (int ind = size - 2; ind >= 0; --ind) {
    EpochObservation tmp_obs = baser_obs_[ind];
    double tmp = tmp_obs.gnss_week() * sec_per_week + tmp_obs.gnss_second_s();
    tmp -= t_rover;
    if (fabs(tmp) < fabs(tgap)) {
      tgap = tmp;
      *baser_obs = tmp_obs;
    } else if (fabs(tmp) > fabs(tgap)) {
      break;
    }
    if (fabs(tmp) < gap_threshold) {
      break;
    }
  }
  return true;
}

int GnssPntSolver::UpdateBaseCoor(const EpochObservation& rover_obs,
                                    const EpochObservation& baser_obs) {
  // for real time application, consider baser coordinate update
  double dis_gap = 0.0;
  bool valid_base_coor = baser_obs.has_position_x() &&
                         baser_obs.has_position_y() &&
                         baser_obs.has_position_z();
  if (!valid_base_coor) {
    return 0;
  }

  PointThreeDim temp(baser_obs.position_x(), baser_obs.position_y(),
                     baser_obs.position_z());
  dis_gap = gnss_utility::GetDistance(temp, baser_coor_);
  if (dis_gap >= 0.03) {
    SetBaserCoordinate(baser_obs.position_x(), baser_obs.position_y(),
                         baser_obs.position_z());
    // TO DO: a more specific step would be employed to keep AR-Fiexed
    // during base data source changing.
    ResetFixedRtk();
  }
  return 1;
}

int GnssPntSolver::Solve(const EpochObservation& rover_obs,
                         GnssPntResult* rover_pnt) {
  apollo::localization::local_gnss::EpochObservation baser_obs;
  baser_obs.Clear();
  bool syn_succ =
      SynchronizeBaser(rover_obs.gnss_week(), rover_obs.gnss_second_s(),
                        &baser_obs, synch_time_gap_threshold_);
  UpdateBaseCoor(rover_obs, baser_obs);
  return SolveWithBaser(rover_obs, baser_obs, syn_succ, rover_pnt);
}

int GnssPntSolver::SolveWithBaser(
    const EpochObservation& rover_obs,
    const EpochObservation& baser_obs,
    const bool& valid_base,
    GnssPntResult* rover_pnt) {
  // initialize
  invalid_rtk_sub_infor_ = "";
  rover_pnt->Clear();
  rover_pnt->set_receiver_id(rover_obs.receiver_id());
  rover_pnt->set_gnss_week(rover_obs.gnss_week());
  rover_pnt->set_gnss_second_s(rover_obs.gnss_second_s());
  rover_pnt->set_pnt_type(PNT_INVALID);

  double gnss_time = rover_obs.gnss_week() * 604800 + rover_obs.gnss_second_s();

  PointThreeDim spp_result = rover_coor_;
  std::vector<SatelliteInfor> rover_sat_vector_used;
  std::vector<SatelliteInfor> baser_sat_vector_used;
  std::vector<GnssType> rover_related_gnss_type;

  const double spp_ure = 3.0 * 3.0;
  apollo::localization::local_gnss::PointThreeDim spp_dop(999.9, 999.9, 999.9);
  double std_rover =
      GetStandardPosition(rover_obs, &rover_sat_vector_used,
                        &rover_related_gnss_type, &spp_result, &spp_dop);

  double pdop = sqrt(spp_dop.x + spp_dop.y + spp_dop.z);
  if (!IsSafeStandardPointPos(std_rover, pdop)) {
    ResetFixedRtk();
    rover_preprocessor_.ReInitialize();
    return RET_BAD_SPP;
  }

  PointThreeDim spp_std = spp_dop * spp_ure;
  spp_std = spp_std.SquareRoot();
  rover_pnt->set_pos_x_m(spp_result.x);
  rover_pnt->set_pos_y_m(spp_result.y);
  rover_pnt->set_pos_z_m(spp_result.z);
  rover_pnt->set_std_pos_x_m(spp_std.x);
  rover_pnt->set_std_pos_y_m(spp_std.y);
  rover_pnt->set_std_pos_z_m(spp_std.z);
  rover_pnt->set_sovled_sat_num(rover_sat_vector_used.size());
  // set valid_spp
  rover_pnt->set_pnt_type(PNT_SPP);
  rover_pnt->set_pdop(pdop);

  // single point velocity using doppler
  PointThreeDim vel_xyz;
  PointThreeDim vel_rover(-999.9, -999.9, -999.9);
  // velocity from delta_p / delta_time
  double std_dopper = GetStandardVelocity(rover_obs, rover_sat_vector_used,
                                        rover_related_gnss_type, &vel_xyz);
  // bool valid_vel_doppler = false;
  if (std_dopper > 0.0 && std_dopper < 3.0) {
    gnss_utility::dxyz2enu(spp_result, vel_xyz, &vel_rover);
    rover_pnt->set_vel_x_m(vel_rover.x);
    rover_pnt->set_vel_y_m(vel_rover.y);
    rover_pnt->set_vel_z_m(vel_rover.z);
    PointThreeDim vel_std = std_doppler_diff_.SquareRoot();
    rover_pnt->set_std_vel_x_m(vel_std.x);
    rover_pnt->set_std_vel_y_m(vel_std.y);
    rover_pnt->set_std_vel_z_m(vel_std.z);
    // valid_vel_doppler = true;
  }

  if (position_option_ <= apollo::localization::local_gnss::STANDARD) {
    return RET_SPP_MODE;
  }

  rover_coor_ = spp_result;
  DeleteElvCutoffSat(elevation_cutoff_deg_, &rover_sat_vector_used);

  PointThreeDim std_slip_rover(-999.9, -999.9, -999.9);

  bool valid_delta_pos = false;
  PointThreeDim ptd_result = spp_result;
  valid_time_diff_ = rover_preprocessor_.DetectBasedOnGeometry(
      rover_related_gnss_type, ptd_result, rover_obs, rover_sat_vector_used);
  if (valid_time_diff_) {
    // average velocity
    vel_rover = rover_preprocessor_.GetVelocity();
    time_diff_delta_pos_ = rover_preprocessor_.GetDeltaPos();
    // if (!valid_vel_doppler) {
    rover_pnt->set_vel_x_m(vel_rover.x);
    rover_pnt->set_vel_y_m(vel_rover.y);
    rover_pnt->set_vel_z_m(vel_rover.z);
    std_slip_rover = rover_preprocessor_.GetVelocityStd();
    rover_pnt->set_std_vel_x_m(std_slip_rover.x);
    rover_pnt->set_std_vel_y_m(std_slip_rover.y);
    rover_pnt->set_std_vel_z_m(std_slip_rover.z);
    //}
    if (enable_td_phase_update_) {
      // delta position
      valid_delta_pos = rover_preprocessor_.IsPreciseSet();
    }
    if (valid_delta_pos) {
      rover_pnt->set_pnt_type(PNT_PHASE_TD);
      rover_pnt->set_pos_x_m(ptd_result.x);
      rover_pnt->set_pos_y_m(ptd_result.y);
      rover_pnt->set_pos_z_m(ptd_result.z);
      rover_pnt->set_sovled_sat_num(rover_sat_vector_used.size());
      std_slip_rover = rover_preprocessor_.GetPositionStd();
      rover_pnt->set_std_pos_x_m(std_slip_rover.x);
      rover_pnt->set_std_pos_y_m(std_slip_rover.y);
      rover_pnt->set_std_pos_z_m(std_slip_rover.z);
      rover_coor_ = ptd_result;
      spp_result = ptd_result;
    }
  }

  // NOTICE: core setup for external aiding, not supported yet.
  test_ins_aid_ = false;

  if (position_option_ <= apollo::localization::local_gnss::TIME_DIFF_PHASE) {
    return RET_TIME_DIFF_PHASE_MODE;
  }

  if (!valid_base) {
    return RET_NO_VALID_BASER;
  }

  // for real time, consider baser coordinate update
  double dis_gap = 0.0;
  // when basercoor not set
  if (baser_coor_.Norm3D() < 30000.0) {
    EncodeDetails("base coordinate = %12.3f %12.3f %12.3f", baser_coor_.x,
                   baser_coor_.y, baser_coor_.z);
    return RET_INVALID_BASE_COORDINATE;
  }
  PointThreeDim temp_baser = spp_result;
  apollo::localization::local_gnss::PointThreeDim baser_pos_spp;
  std::vector<GnssType> baser_related_gnss_type;
  double std_baser =
      GetStandardPosition(baser_obs, &baser_sat_vector_used,
                        &baser_related_gnss_type, &temp_baser, &spp_dop);

  dis_gap = gnss_utility::GetDistance(temp_baser, baser_coor_);
  if (!CheckBaserObs(std_baser, dis_gap)) {
    ResetFixedRtk();
    EncodeDetails("base spp std = %6.4f and its gap with fixed coor = %12.3f",
                   std_baser, dis_gap);
    return RET_INVALID_BASE_SPP;
  }

  // when diff_age exceeds threshold, 60 seconds, no code-diff supported
  double data_outage = rover_obs.gnss_second_s() - baser_obs.gnss_second_s();
  distance_rover2baser_ = gnss_utility::GetDistance(baser_coor_, spp_result);
  if (!IsEnabledCodeDiff(data_outage, distance_rover2baser_)) {
    ResetFixedRtk();
    EncodeDetails("data_outage = %.2f  dis2base = %.2f", data_outage,
                   distance_rover2baser_);
    return RET_BIG_BASE_DATA_GAP;
  }

  PointThreeDim vel_baser(999.9, 999.9, 999.9);
  if (baser_preprocessor_.DetectBasedOnGeometry(baser_related_gnss_type,
                                                temp_baser, baser_obs,
                                                baser_sat_vector_used)) {
    vel_baser = baser_preprocessor_.GetVelocity();
    // TO DO: not suitable for moving dual-antenna mode.
    if (vel_baser.Norm3D() > 0.2) {
      ResetFixedRtk();
    }
  }

  if (!external_fix_baser_) {
    // without external settings
    baser_coor_ = temp_baser;
  }

  std::vector<SatelliteInfor> common_in_rover;
  std::vector<SatelliteInfor> common_in_baser;
  std::vector<GnssType> related_gnss_type;
  SelectCommonSatellite(rover_sat_vector_used, baser_sat_vector_used,
                          baser_coor_,
                          &common_in_rover, &common_in_baser,
                          &related_gnss_type);
  if (common_in_baser.size() < 6) {
    ResetFixedRtk();
    rover_preprocessor_.ReInitialize();
    EncodeDetails("common satellites number = %4d", common_in_baser.size());
    return RET_NOT_ENOUGH_COMMON_SAT;
  }
  // code difference
  range_precision_ = 2 * 1.0 * 1.0;
  double std_dif = std_rover;
  PointThreeDim code_dif_result = spp_result;
  if (GetCodeDiffPosition(rover_obs, baser_obs,
                         common_in_baser,
                         related_gnss_type,
                         &common_in_rover,
                         &code_dif_result)) {
    PointThreeDim dif_xyz = code_dif_result - spp_result;
    PointThreeDim dif_enu = dif_xyz;
    gnss_utility::dxyz2enu(spp_result, dif_xyz, &dif_enu);
    double dis = sqrt(dif_enu.x * dif_enu.x + dif_enu.y * dif_enu.y);
    std_dif = std_code_diff_.Norm3D();
    if (IsSafeCodeDiffPos(dis, std_dif, valid_delta_pos)) {
      rover_pnt->set_pnt_type(PNT_CODE_DIFF);
      rover_pnt->set_pos_x_m(code_dif_result.x);
      rover_pnt->set_pos_y_m(code_dif_result.y);
      rover_pnt->set_pos_z_m(code_dif_result.z);
      rover_pnt->set_std_pos_x_m(std_code_diff_.x);
      rover_pnt->set_std_pos_y_m(std_code_diff_.y);
      rover_pnt->set_std_pos_z_m(std_code_diff_.z);
      rover_pnt->set_sovled_sat_num(common_in_rover.size());
      spp_result = code_dif_result;
    }
  }
  BoundRangePrecision(std_dif);

  if (position_option_ <= apollo::localization::local_gnss::CODE_DIFF) {
    return RET_CODE_DIFF_MODE;
  }
  // diff_age exceeds 15 seconds (currently happpen under ntrip caster)
  if (!IsEnabledPhaseDiff(data_outage, distance_rover2baser_)) {
    ResetFixedRtk();
    EncodeDetails("data_outage = %.2f  dis2base = %.2f", data_outage,
                   distance_rover2baser_);
    return RET_BIG_BASE_DATA_GAP_RTK;
  }

  for (unsigned int i = 0; i < common_in_baser.size(); ++i) {
    common_in_baser[i].distance = gnss_utility::GetCosineDirection(
        baser_coor_, common_in_baser[i].position,
        &common_in_baser[i].direction);
    common_in_rover[i].distance = gnss_utility::GetCosineDirection(
        spp_result, common_in_rover[i].position, &common_in_rover[i].direction);
  }

  // phase-diff(RTK)
  PointThreeDim rtk_result = spp_result;
  if (!GetPhaseDiffPosition(rover_obs, baser_obs, common_in_rover,
                           common_in_baser, related_gnss_type, baser_coor_,
                           &rtk_result)) {
    ResetFixedRtk();
    return RET_INVALID_RTK;
  }

  PointThreeDim phase_std = BoundedRtkStd();
  if (IsSafeFixedSolution(related_gnss_type.size(),
                          amb_tracker_.GetNumFixedAmb())) {
    spp_result = rtk_result;
    rover_pnt->set_sovled_sat_num(common_in_rover.size());
    rover_pnt->set_pos_x_m(spp_result.x);
    rover_pnt->set_pos_y_m(spp_result.y);
    rover_pnt->set_pos_z_m(spp_result.z);
    rover_pnt->set_std_pos_x_m(phase_std.x);
    rover_pnt->set_std_pos_y_m(phase_std.y);
    rover_pnt->set_std_pos_z_m(phase_std.z);
    rover_coor_ = spp_result;
    CaculateHeading(baser_coor_, rover_coor_, phase_std);
    rover_pnt->set_pnt_type(PNT_RTK_FIXED);
    last_fixed_rover_pnt_ = *rover_pnt;
    // enable phase-time-diff absolute position when possible.
    if (amb_tracker_.GetNumFixedAmb() > 10) {
      rover_preprocessor_.SetPresiceCoor(gnss_time, spp_result, phase_std);
    }

  } else {
    // rover_pnt->set_pnt_type(PNT_RTK_FLOAT);
  }

  if (position_option_ <= apollo::localization::local_gnss::RTK) {
    return RET_VALID_RTK;
  }

  return RET_BAD_SPP;
}

double GnssPntSolver::GetStandardPosition(
    const EpochObservation obs,
    std::vector<SatelliteInfor>* sate_vector_used,
    std::vector<GnssType>* related_gnss_type,
    PointThreeDim* spp_result,
    PointThreeDim* spp_dop) {
  double deg2rad = apollo::localization::local_gnss::PI / 180.0;
  const unsigned int max_iterate_num = 10;
  unsigned int num_iterate = 0;
  const double ellipson = 0.1;

  // pending 3 virtual position observations
  Eigen::MatrixXd cc(obs.sat_obs_num() + 3, 3);
  Eigen::MatrixXd bb(obs.sat_obs_num() + 3, max_gnss_sys_num_);
  Eigen::MatrixXd prior_res(obs.sat_obs_num() + 3, 1);
  Eigen::MatrixXd clk(obs.sat_obs_num() + 3, 1);
  Eigen::MatrixXd post_res(obs.sat_obs_num() + 3, 1);
  post_res.setZero();
  related_gnss_type->resize(0);

  double std_res = 10000000.0;
  unsigned int num_valid_sat = 0;
  unsigned int num_valid_obs = 0;

  Eigen::MatrixXd receiver_clk(max_gnss_sys_num_, 1);
  receiver_clk.setZero();

  Eigen::MatrixXd dx;
  while (num_iterate < max_iterate_num) {
    ++num_iterate;
    sate_vector_used->resize(0);
    cc.setZero();
    bb.setZero();
    prior_res.setZero();
    clk.setZero();
    num_valid_sat = 0;
    num_valid_obs = 0;
    Atmosphere trop_rover(*spp_result, obs.gnss_week(), obs.gnss_second_s());
    for (unsigned int i = 0; i < obs.sat_obs_num(); ++i) {
      if (!CheckSatObsForSPP(obs.sat_obs(i))) {
        continue;
      }
      double time_duration =
          obs.sat_obs(i).band_obs(0).pseudo_range() / C_MPS;
      double time_begin = obs.gnss_second_s() - time_duration;
      double clk_error = 0.0;
      // get receiver clock error of certain navigation system
      for (unsigned int m = 0; m < related_gnss_type->size(); ++m) {
        if (obs.sat_obs(i).sat_sys() == (*related_gnss_type)[m]) {
          clk_error = receiver_clk(m, 0) / C_MPS;
          break;
        }
      }
      SatelliteInfor sat_pvt;
      sat_pvt.time_travles = time_duration - clk_error;
      sat_pvt.time_signal_transmitted = time_begin;
      sat_pvt.gnss_week = obs.gnss_week();
      // toe set -0.1 to query eph using observing time otherwise using explicit
      // toe!
      sat_pvt.toe = -0.1;
      if (global_ephemeris_.GetPosVelClock(
              obs.sat_obs(i).sat_sys(), obs.sat_obs(i).sat_prn(),
              obs.gnss_week(), time_begin, sat_pvt.time_travles,
              &sat_pvt.position, &sat_pvt.velocity, &sat_pvt.clock_bias,
              &sat_pvt.clock_drift, &sat_pvt.toe) == false) {
        continue;
      }

      unsigned int j = 0;
      for (j = 0; j < related_gnss_type->size(); j++) {
        if (obs.sat_obs(i).sat_sys() == (*related_gnss_type)[j]) {
          break;
        }
      }
      if (j == related_gnss_type->size()) {
        related_gnss_type->push_back(obs.sat_obs(i).sat_sys());
      }
      bb(num_valid_sat, j) = 1;
      sat_pvt.distance = gnss_utility::GetCosineDirection(
          *spp_result, sat_pvt.position, &sat_pvt.direction);

      apollo::localization::local_gnss::gnss_utility::GetEleAzm(
          *spp_result, sat_pvt.position, &sat_pvt.elevation, &sat_pvt.azimuth);
      sat_pvt.index_in_obs = i;
      sat_pvt.sat_sys = obs.sat_obs(i).sat_sys();
      sat_pvt.sat_prn = obs.sat_obs(i).sat_prn();

      cc(num_valid_sat, 0) = sat_pvt.direction.x;
      cc(num_valid_sat, 1) = sat_pvt.direction.y;
      cc(num_valid_sat, 2) = sat_pvt.direction.z;
      prior_res(num_valid_sat, 0) =
          obs.sat_obs(i).band_obs(0).pseudo_range() - sat_pvt.distance;
      clk(num_valid_sat, 0) =
          sat_pvt.clock_bias * apollo::localization::local_gnss::C_MPS;
      prior_res(num_valid_sat, 0) += clk(num_valid_sat, 0);

      double rover_trop_delay = trop_rover.TropoDelay(
          sat_pvt.elevation * deg2rad, sat_pvt.azimuth * deg2rad);
      prior_res(num_valid_sat, 0) -= rover_trop_delay;
      ++num_valid_sat;
      ++num_valid_obs;
      sate_vector_used->push_back(sat_pvt);
    }
    if (num_valid_sat <= 3 + related_gnss_type->size()) {
      // To do: add interface for prior setting.
      return -0.1;
    }
    // combine position and clock coefficients
    Eigen::MatrixXd aa(num_valid_obs, 3 + related_gnss_type->size());
    Eigen::MatrixXd vv(num_valid_obs, 1);
    Eigen::MatrixXd pp(num_valid_obs, num_valid_obs);
    aa.setZero();
    vv.setZero();
    pp.setIdentity();
    for (unsigned int i = 0; i < num_valid_obs; ++i) {
      for (unsigned int j = 0; j < 3; ++j) {
        aa(i, j) = cc(i, j);
      }
      if (i < num_valid_sat) {
        for (unsigned int j = 0; j < related_gnss_type->size(); ++j) {
          aa(i, j + 3) = bb(i, j);
        }
      }
      vv(i, 0) = prior_res(i, 0);
    }
    // To do: equation solving shall cover situation without enough satellites
    Eigen::MatrixXd at = aa.transpose();
    Eigen::MatrixXd ata = at * pp * aa;
    Eigen::MatrixXd atv = at * pp * vv;
    Eigen::MatrixXd inv_atpa = ata.inverse();

    dx = inv_atpa * atv;
    post_res = vv - aa * dx;
    Eigen::MatrixXd temp = post_res.transpose() * pp * post_res;
    double delta =
        temp(0, 0) / (num_valid_sat - 3 - related_gnss_type->size() + 1);
    for (unsigned int i = 0; i < num_valid_obs; ++i) {
      pp(i, i) = delta / pow(fabs(post_res(i, 0)) + 1.0, 3.0);
      if (pp(i, i) > 10000) {
        pp(i, i) = 10000;
      }
    }
    ata = at * pp * aa;
    atv = at * pp * vv;
    inv_atpa = ata.inverse();
    dx = inv_atpa * atv;
    post_res = vv - aa * dx;
    // update receiver clock
    for (unsigned int i = 3; i < dx.rows(); ++i) {
      receiver_clk(i - 3, 0) = dx(i, 0);
    }
    Eigen::MatrixXd vtpv = post_res.transpose() * pp * post_res /
                           (num_valid_sat - 3 - related_gnss_type->size());
    Eigen::MatrixXd test_v = inv_atpa * vtpv(0, 0);
    // PrintMatrix(test_v, "test_v");
    std_res = sqrt(test_v(0, 0) + test_v(1, 1) + test_v(2, 2));
    // shield satellites with gross post residual
    for (unsigned int i = 0; i < post_res.rows(); ++i) {
      if (fabs(post_res(i, 0)) > 4.0 * std_res &&
          fabs(post_res(i, 0)) >= 15.0) {
        (*sate_vector_used)[i].multi_path = true;
      }
    }
    spp_result->x += dx(0, 0);
    spp_result->y += dx(1, 0);
    spp_result->z += dx(2, 0);

    Eigen::MatrixXd dop = at * aa;
    dop = dop.inverse();
    spp_dop->x = dop(0, 0);
    spp_dop->y = dop(1, 1);
    spp_dop->z = dop(2, 2);
    if (fabs(dx(0, 0)) < ellipson && fabs(dx(1, 0)) < ellipson &&
        fabs(dx(2, 0)) < ellipson) {
      break;
    }
  }
  unsigned int size_current = sate_vector_used->size();
  for (unsigned int j = 0; j < size_current; ++j) {
    (*sate_vector_used)[j].distance =
        apollo::localization::local_gnss::gnss_utility::GetDistance(
            *spp_result, (*sate_vector_used)[j].position);
  }
  return std_res;
}

double GnssPntSolver::GetStandardVelocity(
    const EpochObservation& obs_rover,
    const std::vector<SatelliteInfor>& sate_vector_used,
    const std::vector<GnssType>& related_gnss_type, PointThreeDim* spv_result) {
  const unsigned int sat_num = sate_vector_used.size();
  unsigned int sys_num = related_gnss_type.size();

  const unsigned int max_iterate_num = 5;
  unsigned int num_iterate = 0;
  const double ellipson = 0.01;

  Eigen::MatrixXd v;
  Eigen::MatrixXd a;
  Eigen::MatrixXd p;

  Eigen::MatrixXd vv(sat_num, 1);
  Eigen::MatrixXd aa(sat_num, 3 + sys_num);
  Eigen::MatrixXd pp(sat_num, sat_num);

  unsigned int doppler_obs_num = 0;
  double std = 0.0;
  // calculating Doppler
  std::vector<double> doppler_cal;
  doppler_cal.resize(sat_num);
  while (num_iterate < max_iterate_num) {
    ++num_iterate;
    doppler_obs_num = 0;
    vv.setZero();
    aa.setZero();
    pp.setIdentity(sat_num, sat_num);
    for (unsigned int i = 0; i < sat_num; ++i) {
      double r_s =
          sate_vector_used[i].direction.x * sate_vector_used[i].velocity.x +
          sate_vector_used[i].direction.y * sate_vector_used[i].velocity.y +
          sate_vector_used[i].direction.z * sate_vector_used[i].velocity.z;

      double r_c = sate_vector_used[i].direction.x * spv_result->x +
                   sate_vector_used[i].direction.y * spv_result->y +
                   sate_vector_used[i].direction.z * spv_result->z;
      doppler_cal[i] = (r_s - r_c);
      // get rover band obs
      BandObservation rover_band_j;
      assert(sate_vector_used[i].index_in_obs >= 0);
      rover_band_j =
          obs_rover.sat_obs(sate_vector_used[i].index_in_obs).band_obs(0);
      int glo_fre_num = 0;
      double rover_band_j_len = GetBandLength(
          sate_vector_used[i].sat_prn, rover_band_j.band_id(), &glo_fre_num);

      unsigned int j = 0;
      for (j = 0; j < sys_num; j++) {
        if (sate_vector_used[i].sat_sys == related_gnss_type[j]) {
          break;
        }
      }
      if (!rover_band_j.has_doppler()) {
        continue;
      }
      ++doppler_obs_num;
      aa(i, j + 3) = 1;
      aa(i, 0) = sate_vector_used[i].direction.x;
      aa(i, 1) = sate_vector_used[i].direction.y;
      aa(i, 2) = sate_vector_used[i].direction.z;
      // 1 for rtcm, -1 for novatel
      int sign = -1.0;
      vv(i, 0) =
          sign * rover_band_j.doppler() * rover_band_j_len - (-doppler_cal[i]);
      vv(i, 0) +=
          sate_vector_used[i].clock_drift *
          apollo::localization::local_gnss::C_MPS;
      double sina = sin(sate_vector_used[i].elevation / 180.0 * PI);
      pp(i, i) = sina;
      if (sate_vector_used[i].multi_path) {
        pp(i, i) *= 0.05;
      }
    }
    if (doppler_obs_num <= 3 + related_gnss_type.size()) {
      return -0.1;
    }
    a = aa.topRows(doppler_obs_num);
    v = vv.topRows(doppler_obs_num);
    p = pp.topLeftCorner(doppler_obs_num, doppler_obs_num);
    Eigen::MatrixXd atp = a.transpose() * p;
    Eigen::MatrixXd atpa = atp * a;
    Eigen::MatrixXd atpa_inv = atpa.inverse();
    Eigen::MatrixXd dx = atpa_inv * (atp * v);
    spv_result->x += dx(0, 0);
    spv_result->y += dx(1, 0);
    spv_result->z += dx(2, 0);
    Eigen::MatrixXd res = v - a * dx;

    Eigen::MatrixXd vtpv = res.transpose() * p * res /
                           (doppler_obs_num - 3 - related_gnss_type.size());
    Eigen::MatrixXd test_v = atpa_inv * vtpv(0, 0);
    // PrintMatrix(test_v, "test_v");
    std = sqrt(test_v(0, 0) + test_v(1, 1) + test_v(2, 2));
    std_doppler_diff_.x = test_v(0, 0);
    std_doppler_diff_.y = test_v(1, 1);
    std_doppler_diff_.z = test_v(2, 2);
    if (fabs(dx(0, 0)) < ellipson && fabs(dx(1, 0)) < ellipson &&
        fabs(dx(2, 0)) < ellipson) {
      break;
    }
  }
  return std;
}

bool GnssPntSolver::GetCodeDiffPosition(
    const EpochObservation& obs_rover,
    const EpochObservation& obs_baser,
    const std::vector<SatelliteInfor>& common_in_baser,
    const std::vector<GnssType>& related_gnss_type,
    std::vector<SatelliteInfor>* common_in_rover,
    PointThreeDim* code_dif_result) {
  const unsigned int common_sat_num = common_in_rover->size();
  unsigned int sys_num = related_gnss_type.size();

  if (common_sat_num <= 3 + sys_num) {
    return false;
  }

  const unsigned int max_iterate_num = 10;
  unsigned int num_iterate = 0;
  const double ellipson = 0.01;
  double std_res = 0.0;

  unsigned int num_constrains = 0;

  Eigen::MatrixXd vv(common_sat_num + num_constrains, 1);
  Eigen::MatrixXd aa(common_sat_num + num_constrains, 3 + sys_num);
  Eigen::MatrixXd res(common_sat_num + num_constrains, 1);
  while (num_iterate < max_iterate_num) {
    ++num_iterate;
    vv.setZero();
    aa.setZero();
    res.setZero();
    for (unsigned int i = 0; i < common_sat_num; ++i) {
      // get rover band obs
      BandObservation rover_band_j;
      assert((*common_in_rover)[i].index_in_obs >= 0);
      rover_band_j =
          obs_rover.sat_obs((*common_in_rover)[i].index_in_obs).band_obs(0);

      // update rover distance and direction cosine
      (*common_in_rover)[i].distance = gnss_utility::GetCosineDirection(
          *code_dif_result, (*common_in_rover)[i].position,
          &(*common_in_rover)[i].direction);
      // get baser band obs
      BandObservation baser_band_j;
      assert(common_in_baser[i].index_in_obs >= 0);
      baser_band_j =
          obs_baser.sat_obs(common_in_baser[i].index_in_obs).band_obs(0);

      unsigned int j = 0;
      for (j = 0; j < sys_num; j++) {
        if ((*common_in_rover)[i].sat_sys == related_gnss_type[j]) {
          break;
        }
      }
      aa(i, 3 + j) = 1;
      aa(i, 0) = (*common_in_rover)[i].direction.x;
      aa(i, 1) = (*common_in_rover)[i].direction.y;
      aa(i, 2) = (*common_in_rover)[i].direction.z;
      vv(i, 0) = rover_band_j.pseudo_range() - baser_band_j.pseudo_range() -
                 ((*common_in_rover)[i].distance - common_in_baser[i].distance);
      vv(i, 0) +=
          ((*common_in_rover)[i].clock_bias - common_in_baser[i].clock_bias) *
          apollo::localization::local_gnss::C_MPS;
    }
    Eigen::MatrixXd atpa_inv = (aa.transpose() * aa).inverse();
    Eigen::MatrixXd dx = atpa_inv * (aa.transpose() * vv);
    res = vv - aa * dx;
    Eigen::MatrixXd vtpv = res.transpose() * res /
                           (res.rows() - (3 + related_gnss_type.size()) + 1);
    Eigen::MatrixXd std_3d = atpa_inv * vtpv(0, 0);
    // PrintMatrix(vtpv, "vtpv");
    std_res = sqrt(std_3d(0, 0) + std_3d(1, 1) + std_3d(2, 2));
    std_code_diff_.x = sqrt(std_3d(0, 0));
    std_code_diff_.y = sqrt(std_3d(1, 1));
    std_code_diff_.z = sqrt(std_3d(2, 2));

    code_dif_result->x += dx(0, 0);
    code_dif_result->y += dx(1, 0);
    code_dif_result->z += dx(2, 0);
    if (fabs(dx(0, 0)) < ellipson && fabs(dx(1, 0)) < ellipson &&
        fabs(dx(2, 0)) < ellipson) {
      break;
    }
  }
  if (fabs(std_res) > 10.0) {
    return false;
  }
  return true;
}

bool GnssPntSolver::GetGnssSystemToSolve() {
  unsigned int size_band = band_to_solve_.size();
  if (size_band <= 0) {
    return false;
  }
  gnss_to_solve_.resize(0);
  for (unsigned int i = 0; i < size_band; ++i) {
    GnssType temp = global_ephemeris_.DetermineGnssType(band_to_solve_[i]);
    if (temp == apollo::drivers::gnss::SYS_UNKNOWN) {
      continue;
    }
    if (gnss_utility::GetIndexInVector(temp, gnss_to_solve_) == -1) {
      gnss_to_solve_.push_back(temp);
    }
  }
  return gnss_to_solve_.size() > 0;
}

bool GnssPntSolver::GetPhaseDiffPosition(
    const EpochObservation& obs_rover,
    const EpochObservation& obs_baser,
    const std::vector<SatelliteInfor>& common_in_rover,
    const std::vector<SatelliteInfor>& common_in_baser,
    const std::vector<GnssType>& related_gnss_type,
    const PointThreeDim& baser_coor,
    PointThreeDim* phasedif_result) {
  CheckConsecutiveTimeOffset(obs_rover.gnss_week(),
                                obs_rover.gnss_second_s());
  CheckSlipOfUnfixedPhase();

  num_sd_phase_recursive_ = 0;
  return ConstructSDEquation(obs_rover, obs_baser, common_in_rover,
                               common_in_baser, related_gnss_type, baser_coor,
                               phasedif_result);
}

bool GnssPntSolver::CheckSlip(
    const apollo::drivers::gnss::GnssBandID& band_id,
    const unsigned int sat_prn, const unsigned int loss_lock_index_rover,
    const unsigned int loss_lock_index_baser) {
  double slip_rover = 0.0;
  double slip_baser = 0.0;
  if (!rover_preprocessor_.CheckSlipValue(band_id, sat_prn, &slip_rover) &&
      !baser_preprocessor_.CheckSlipValue(band_id, sat_prn, &slip_baser)) {
    return true;
  }
  double slip_res_threshold = 0.1;
  if (GetCycleSlipFix()) {
    double scale = 1.0;
    if (loss_lock_index_rover == 2 || loss_lock_index_baser == 2) {
      scale = 2.0;
    }
    double sd_slip = (slip_rover - slip_baser) * scale;
    double int_sd_slip = RoundDouble(sd_slip);
    if (fabs(int_sd_slip - sd_slip) < slip_res_threshold / scale) {
      double amb = 0.0;
      if (amb_tracker_.GetFixedAmb(band_id, sat_prn, &amb)) {
        amb_tracker_.SetFixedAmb(band_id, sat_prn, amb + int_sd_slip / scale);
      }
    } else {
      // reset fixed amb to re-estimate again
      amb_tracker_.DeleteSlipAmb(band_id, sat_prn);
    }
  } else {
    amb_tracker_.DeleteSlipAmb(band_id, sat_prn);
  }
  return false;
}

// old strategy
bool GnssPntSolver::FixJudge(double ratio, const double threshold, double vtpv,
                              double std_phase, const unsigned int valid_num) {
  if (b_rtkfixed_) {
    return true;
  }

  if (vtpv > 10.0 || std_phase > 0.02) {
    good_ratio_ = 0;
    ClearSequential();
    return false;
  }

  if (ratio < threshold) {
    good_ratio_ = 0;
    ratio_ = 0.0;
    return false;
  }

  ratio_ += ratio;
  ++good_ratio_;
  if (good_ratio_ >= valid_num) {
    b_rtkfixed_ = true;
    good_ratio_ = 0;
  }
  return b_rtkfixed_;
}

bool GnssPntSolver::FixJudgeCombined(double ratio, const double threshold,
                                  double vtpv, double std_phase,
                                  double fail_rate) {
  if (fail_rate > 0.05) {
    return false;
  }
  return FixJudge(ratio, threshold, vtpv, std_phase, 3);
}

bool GnssPntSolver::IsEnoughPhaseObs(const unsigned int valid_phase_size,
                                      const unsigned int raw_gnss_type_size,
                                      const unsigned int valid_gnss_type_size) {
  if (valid_phase_size < 4 * raw_gnss_type_size) {
    return false;
  }
  if (valid_gnss_type_size < raw_gnss_type_size) {
    return false;
  }
  return true;
}

bool GnssPntSolver::ConstructSDEquation(
    const EpochObservation& obs_rover, const EpochObservation& obs_baser,
    const std::vector<SatelliteInfor>& common_in_rover,
    const std::vector<SatelliteInfor>& common_in_baser,
    const std::vector<GnssType>& related_gnss_type,
    const PointThreeDim& baser_coor, PointThreeDim* rover_coor) {
  EnableHalfCycleAr(false);
  amb_tracker_.ClearHalfCycleRecorder();
  Atmosphere trop_baser(baser_coor, obs_rover.gnss_week(),
                        obs_rover.gnss_second_s());
  Atmosphere trop_rover(*rover_coor, obs_rover.gnss_week(),
                        obs_rover.gnss_second_s());

  const unsigned int common_sat_num = common_in_rover.size();
  const unsigned int gnss_sys_num = related_gnss_type.size();
  // at most,triple frequencies for range and phase respectively
  const unsigned int max_common_obs_num = common_sat_num * 3 * 2;
  unsigned int num_zwd = 0;
  if (ztd_option_ >= ZTD_ESTIMATE) {
    num_zwd = 1;
  }
  // estimate glonass ifb with different mixed receiver to fix glonass amb.
  unsigned int num_ifb = 0;
  double ifb0 = glonass_ifb_;
  if (EnableGlonassIfbEstimated() && IsGlonassExisted(related_gnss_type)) {
    num_ifb = 1;
  }
  unsigned int num_states = 3 + num_zwd + num_ifb + gnss_sys_num;

  Eigen::MatrixXd am_phase(max_common_obs_num, num_states);
  unsigned int valid_phase_num = 0;

  // pseudo-range
  Eigen::MatrixXd ap(max_common_obs_num, num_states);
  Eigen::MatrixXd vp(max_common_obs_num, 1);
  Eigen::MatrixXd pp(max_common_obs_num, max_common_obs_num);
  // carrier_phase
  Eigen::MatrixXd al(max_common_obs_num, num_states);
  Eigen::MatrixXd vl(max_common_obs_num, 1);
  Eigen::MatrixXd Pl(max_common_obs_num, max_common_obs_num);
  Eigen::MatrixXd cc(max_common_obs_num, max_gnss_sys_num_ * 12 * 3);

  // initilization
  am_phase.setZero();
  ap.setZero();
  vp.setZero();
  pp.setIdentity(max_common_obs_num, max_common_obs_num);
  al.setZero();
  vl.setZero();
  Pl.setIdentity(max_common_obs_num, max_common_obs_num);
  cc.setZero();

  // the phase observation with amb resolved turns
  // into a range observation,and common_range_num ++
  unsigned int common_range_num = 0;

  // common_phase_num equals number of unfixed ambiguity
  unsigned int common_phase_num = 0;

  // record valid GnssType to avoid singularity
  std::vector<GnssType> common_related_gnss_type;
  common_related_gnss_type.resize(0);

  // store the range type with 0 for pseudorange and 1 for phase
  std::vector<unsigned int> range_type;
  range_type.resize(0);
  std::vector<apollo::localization::local_gnss::ObsKey> phase_recorder;
  phase_recorder.resize(0);

  num_valid_sat_ = 0;
  amb_tracker_.ClearObsHistory();
  double deg2rad = apollo::localization::local_gnss::PI / 180.0;
  for (unsigned int i = 0; i < common_sat_num; ++i) {
    double baser_trop_delay =
        trop_baser.TropoDelay(common_in_baser[i].elevation * deg2rad,
                               common_in_baser[i].azimuth * deg2rad);
    double rover_trop_delay =
        trop_rover.TropoDelay(common_in_rover[i].elevation * deg2rad,
                               common_in_rover[i].azimuth * deg2rad);
    // estimate the ZWD residual after model correction
    double rover_map_wet =
        trop_rover.GetMapWet(common_in_rover[i].elevation * deg2rad,
                           common_in_rover[i].azimuth * deg2rad);
    // get common sat on rover and baser
    SatelliteObservation rover_sat_j;
    SatelliteObservation baser_sat_j;
    assert(common_in_rover[i].index_in_obs >= 0 &&
           common_in_baser[i].index_in_obs >= 0);
    rover_sat_j = obs_rover.sat_obs(common_in_rover[i].index_in_obs);
    baser_sat_j = obs_baser.sat_obs(common_in_baser[i].index_in_obs);

    // index of clock error coefficient
    int sat_index_in_sys = gnss_utility::GetIndexInVector(
        common_in_rover[i].sat_sys, related_gnss_type);
    if (sat_index_in_sys == -1) {
      continue;
    }
    bool sat_valid = false;
    bool range_valid = false;
    // every band of GNSS in pre-defined settings construct the equation
    for (unsigned int m = 0; m < rover_sat_j.band_obs_num(); ++m) {
      for (unsigned int n = 0; n < baser_sat_j.band_obs_num(); ++n) {
        if (IsValidBandObs(rover_sat_j.band_obs(m), baser_sat_j.band_obs(n)) <
            0) {
          continue;
        }
        sat_valid = true;
        // avoid singularity when one whole system's satellites were excluded by
        // the condition.
        if (gnss_utility::GetIndexInVector(rover_sat_j.sat_sys(),
                                          common_related_gnss_type) == -1) {
          common_related_gnss_type.push_back(rover_sat_j.sat_sys());
        }
        double sina = sin(common_in_rover[i].elevation / 180.0 *
                          apollo::localization::local_gnss::PI);
        int glo_fre_num = 0;
        double len =
            GetBandLength(common_in_rover[i].sat_prn,
                        rover_sat_j.band_obs(m).band_id(), &glo_fre_num);
        double sys_weight = WeightScaleOnGnssSystem(common_in_rover[i].sat_sys);
        // pseudo range process
        if (rover_sat_j.band_obs(m).pseudo_range() > 0.0 &&
              baser_sat_j.band_obs(n).pseudo_range() > 0.0 &&
              fabs(rover_sat_j.band_obs(m).carrier_phase()) > 0.0 &&
              fabs(baser_sat_j.band_obs(n).carrier_phase()) > 0.0 &&
              !range_valid) {
          range_valid = true;
          range_type.push_back(0);
          ap(common_range_num, 0) = common_in_rover[i].direction.x;
          ap(common_range_num, 1) = common_in_rover[i].direction.y;
          ap(common_range_num, 2) = common_in_rover[i].direction.z;
          vp(common_range_num, 0) =
              rover_sat_j.band_obs(m).pseudo_range() -
              baser_sat_j.band_obs(n).pseudo_range() -
              (common_in_rover[i].distance - common_in_baser[i].distance);
          // correct satellite clock bias
          vp(common_range_num, 0) +=
              apollo::localization::local_gnss::C_MPS *
              (common_in_rover[i].clock_bias - common_in_baser[i].clock_bias);
          // To DO: rm wrong range obs with clock bias already corrected
          // clock
          ap(common_range_num, 3 + num_zwd + num_ifb + sat_index_in_sys) = 1;
          if (ztd_option_ >= ZTD_ESTIMATE) {
            ap(common_range_num, 3) = rover_map_wet;
            vp(common_range_num, 0) -= (rover_trop_delay - baser_trop_delay);
          }
          // glo-ifb
          vp(common_range_num, 0) -= glo_fre_num * ifb0;
          if (num_ifb >= 1) {
            ap(common_range_num, 3 + num_zwd) = glo_fre_num;
          }
          pp(common_range_num, common_range_num) = sina / range_precision_;
          pp(common_range_num, common_range_num) /= sys_weight;
          ++common_range_num;
        }
        if (fabs(rover_sat_j.band_obs(m).carrier_phase()) <= 0.0 ||
            fabs(baser_sat_j.band_obs(n).carrier_phase()) <= 0.0) {
          continue;
        }
        ObsKey temp(rover_sat_j.band_obs(m).band_id(),
                                            common_in_rover[i].sat_prn);

        if (rover_sat_j.band_obs(m).has_loss_lock_index()) {
          if (rover_sat_j.band_obs(m).loss_lock_index() == 2) {
#ifdef _NO_SEMI_CYCLE_PHASE_
            continue;
#else
            amb_tracker_.AddHalfCycleRecorder(temp);
            enable_half_cycle_ar_ = true;
#endif
          }
        }
        if (baser_sat_j.band_obs(n).has_loss_lock_index()) {
          if (baser_sat_j.band_obs(n).loss_lock_index() == 2) {
#ifdef _NO_SEMI_CYCLE_PHASE_
            continue;
#else
            amb_tracker_.AddHalfCycleRecorder(temp);
            enable_half_cycle_ar_ = true;
#endif
          }
        }

        // delete fixed amb with slips
        if (!CheckSlip(rover_sat_j.band_obs(m).band_id(),
                        rover_sat_j.sat_prn(),
                        rover_sat_j.band_obs(m).loss_lock_index(),
                        baser_sat_j.band_obs(n).loss_lock_index())) {
          // there exists a slip
          continue;
        }
        // being a tough new sat
        if (IsToughNewSat(temp)) {
          continue;
        }
        range_type.push_back(1);

        phase_recorder.push_back(temp);
        UpdateBandElevation(rover_sat_j.band_obs(m).band_id(),
                         rover_sat_j.sat_prn(), common_in_rover[i].elevation);

        double amb_val = 0.0;
        int ind = -1;
        bool b_fix = GetFixedAmb(rover_sat_j.band_obs(m).band_id(),
                               rover_sat_j.sat_prn(), &amb_val, &ind);
        if (b_fix) {
          ap(common_range_num, 0) = common_in_rover[i].direction.x;
          ap(common_range_num, 1) = common_in_rover[i].direction.y;
          ap(common_range_num, 2) = common_in_rover[i].direction.z;
          vp(common_range_num, 0) =
              rover_sat_j.band_obs(m).carrier_phase() * len -
              baser_sat_j.band_obs(n).carrier_phase() * len -
              (common_in_rover[i].distance - common_in_baser[i].distance);
          // correct satellite clock bias
          vp(common_range_num, 0) +=
              apollo::localization::local_gnss::C_MPS *
              (common_in_rover[i].clock_bias - common_in_baser[i].clock_bias);
          // clock
          ap(common_range_num, 3 + num_zwd + num_ifb + sat_index_in_sys) = 1;
          // zwd
          if (ztd_option_ >= ZTD_ESTIMATE) {
            ap(common_range_num, 3) = rover_map_wet;
            vp(common_range_num, 0) -= (rover_trop_delay - baser_trop_delay);
          }
          // glo-ifb
          vp(common_range_num, 0) -= glo_fre_num * ifb0;
          if (num_ifb >= 1) {
            ap(common_range_num, 3 + num_zwd) = glo_fre_num;
          }
          pp(common_range_num, common_range_num) = sina / phase_precision_;
          pp(common_range_num, common_range_num) /= sys_weight;
          vp(common_range_num, 0) -= len * amb_val;
          am_phase.row(valid_phase_num) = ap.row(common_range_num);
          ++common_range_num;
        } else {
          al(common_phase_num, 0) = common_in_rover[i].direction.x;
          al(common_phase_num, 1) = common_in_rover[i].direction.y;
          al(common_phase_num, 2) = common_in_rover[i].direction.z;
          vl(common_phase_num, 0) =
              rover_sat_j.band_obs(m).carrier_phase() * len -
              baser_sat_j.band_obs(n).carrier_phase() * len -
              (common_in_rover[i].distance - common_in_baser[i].distance);
          // correct satellite clock bias
          vl(common_phase_num, 0) +=
              apollo::localization::local_gnss::C_MPS *
              (common_in_rover[i].clock_bias - common_in_baser[i].clock_bias);
          // clock
          al(common_phase_num, 3 + num_zwd + num_ifb + sat_index_in_sys) = 1;
          // zwd
          if (ztd_option_ >= ZTD_ESTIMATE) {
            al(common_phase_num, 3) = rover_map_wet;
            vl(common_phase_num, 0) -= (rover_trop_delay - baser_trop_delay);
          }
          // glo-ifb
          vl(common_phase_num, 0) -= glo_fre_num * ifb0;
          if (num_ifb >= 1) {
            al(common_phase_num, 3 + num_zwd) = glo_fre_num;
          }
          Pl(common_phase_num, common_phase_num) = sina / phase_precision_;
          Pl(common_phase_num, common_phase_num) /= sys_weight;
          cc(common_phase_num, ind) = len;
          am_phase.row(valid_phase_num) = al.row(common_phase_num);
          ++common_phase_num;
        }
        ++valid_phase_num;
        break;
      }
    }
    if (sat_valid == true) {
      ++num_valid_sat_;
    }
  }

  amb_tracker_.DeleteDescendingAmb();
  // calls for enough valid phase number without excluding one whole system's
  // satellites
  if (!IsEnoughPhaseObs(phase_recorder.size(), related_gnss_type.size(),
                         common_related_gnss_type.size())) {
    // TO DO: remove this strategy and make more tests.
    EncodeDetails("No-enough common phase obs:%d < %d",
                   static_cast<int>(phase_recorder.size()),
                   4 * static_cast<int>(related_gnss_type.size()));
    return false;
  }
  unsigned int unknown_phase_num = amb_tracker_.GetNumUnfixedAmb();

  // single difference for pseudo_range and fixed carrier_phase
  Eigen::MatrixXd am = ap.topRows(common_range_num);
  Eigen::MatrixXd lm = vp.topRows(common_range_num);
  Eigen::MatrixXd pm = pp.topLeftCorner(common_range_num, common_range_num);
  Eigen::MatrixXd atpa = am.transpose() * pm * am;
  Eigen::MatrixXd inv_atpa = atpa.inverse();
  Eigen::MatrixXd atpl = am.transpose() * pm * lm;

  Eigen::MatrixXd dx(num_states, 1);
  dx.setZero();
  dx = inv_atpa * atpl;

  // first do a LS to get a dx
  Eigen::MatrixXd dop = am.transpose() * am;
  dop = dop.inverse();
  PointThreeDim rtk_dop(dop(0, 0), dop(1, 1), dop(2, 2));

  const double phase_p = 0.015;
  const double precision_dd_phase = phase_p * 2.0;

  am_phase = am_phase.topRows(valid_phase_num);
  Eigen::MatrixXd dop_phase = am_phase.transpose() * am_phase;
  dop_phase = dop_phase.inverse();
  rtk_dop_phase_ =
      PointThreeDim(dop_phase(0, 0), dop_phase(1, 1), dop_phase(2, 2));
  std_rtk_thres_of_phase_dop_ =
      rtk_dop_phase_ * precision_dd_phase * precision_dd_phase;
  PointThreeDim dop_xyz = rtk_dop_phase_;
  gnss_utility::dxyz2enu(rover_coor_, dop_xyz, &rtk_dop_phase_);

  std_rtk_ = rtk_dop * precision_dd_phase * precision_dd_phase;
  Eigen::MatrixXd post_res = lm - am * dx;
  Eigen::MatrixXd vtpv = post_res.transpose() * pm * post_res;
  vtpv /= (post_res.rows() - num_states + 1);
  Eigen::MatrixXd std_3d = inv_atpa * vtpv(0, 0);
  _std_v = sqrt(std_3d(0, 0) + std_3d(1, 1) + std_3d(2, 2));
  std_rtk_.x = std_3d(0, 0);
  std_rtk_.y = std_3d(1, 1);
  std_rtk_.z = std_3d(2, 2);

  // valid the fixed solution before adding new observations
  bool re_cal =
      IsNeedRecursion(b_rtkfixed_, _std_v, unknown_phase_num, common_phase_num);
  if (re_cal) {
    ClearSequential();
    ++num_sd_phase_recursive_;
    if (num_sd_phase_recursive_ <= 5) {
      invalid_rtk_sub_infor_ = "";
      return ConstructSDEquation(obs_rover, obs_baser, common_in_rover,
                                   common_in_baser, related_gnss_type,
                                   baser_coor, rover_coor);
    } else {
      EncodeDetails("Too many recursions:%d", num_sd_phase_recursive_);
      return false;
    }
  }
  new_fixed_phase_num_ = 0;
  if (unknown_phase_num <= 0) {
    ClearSequential();
    const unsigned int max_band_num = apollo::drivers::gnss::GLO_G3;
    int band_obs_num[max_band_num] = {0};
    double band_res_sum[max_band_num] = {0.0};
    unsigned int ind_phase_print = 0;
    int glo_fre_num = 0;
    for (unsigned int i = 0; i < range_type.size(); ++i) {
      if (range_type[i] == 0) {
        continue;
      }
      ObsKey temp = phase_recorder[ind_phase_print];
      ++ind_phase_print;
      ++band_obs_num[temp.band_id];
      double len = GetBandLength(temp.sat_prn, temp.band_id, &glo_fre_num);
      band_res_sum[temp.band_id] += post_res(i, 0) / len;
    }

    for (unsigned int i = 0; i < max_band_num; ++i) {
      if (band_obs_num[i] != 0) {
        band_res_sum[i] /= band_obs_num[i];
      }
    }

    if (_std_v < 0.03) {
      counter_for_low_ratio_ = 0;
      ratio_ = 999.9;
      rover_coor->x += dx(0, 0);
      rover_coor->y += dx(1, 0);
      rover_coor->z += dx(2, 0);
      auxiliary_infor_ = dx;
      RemoveAmbWithAbnormalRes(phase_recorder, _std_v, range_type, post_res,
                                   band_res_sum);
      return true;
    }
    // identify abnormal residuals
    unsigned int num_itera = 0;
    Eigen::MatrixXd px = pm;
    while (true) {
      ++num_itera;
      WeightOnPostResidual(_std_v, range_type, post_res, &px);
      atpa = am.transpose() * px * am;
      inv_atpa = atpa.inverse();
      atpl = am.transpose() * px * lm;
      dx = inv_atpa * atpl;
      post_res = lm - am * dx;

      vtpv = post_res.transpose() * px * post_res;
      std_3d = inv_atpa * vtpv(0, 0);
      std_3d /= (post_res.rows() - num_states + 1);
      _std_v = sqrt(std_3d(0, 0) + std_3d(1, 1) + std_3d(2, 2));
      std_rtk_.x = std_3d(0, 0);
      std_rtk_.y = std_3d(1, 1);
      std_rtk_.z = std_3d(2, 2);
      if (_std_v < 0.03 || num_itera >= 2) {
        break;
      }
    }
    RemoveAmbWithAbnormalRes(phase_recorder, _std_v, range_type, post_res,
                                 band_res_sum);
    // healthy result
    ratio_ = 999.9;
    rover_coor->x += dx(0, 0);
    rover_coor->y += dx(1, 0);
    rover_coor->z += dx(2, 0);
    auxiliary_infor_ = dx;
    if (_std_v >= 0.10) {
      EncodeDetails("Abnormal std without new phase:%d , std = %4.2f",
                     static_cast<int>(phase_recorder.size()), _std_v);
      return false;
    }
    return true;
  }

  // single difference for un-fixed carrier_phase
  Eigen::MatrixXd an = al.topRows(common_phase_num);
  Eigen::MatrixXd ln = vl.topRows(common_phase_num);
  Eigen::MatrixXd pn = Pl.topLeftCorner(common_phase_num, common_phase_num);
  Eigen::MatrixXd b = cc.topLeftCorner(common_phase_num, unknown_phase_num);

  PrintMatrix(an, "an");
  PrintMatrix(ln, "ln");
  PrintMatrix(pn, "pn");
  PrintMatrix(b, "b");
  /* solve equation LS
  Vm  = [ am  0 ] [x]  -  lm
  Vn    [ an  Bn] [Y]  -  ln
  */
  atpa += an.transpose() * pn * an;
  inv_atpa = atpa.inverse();
  atpl += an.transpose() * pn * ln;

  Eigen::MatrixXd btpb = b.transpose() * pn * b;
  Eigen::MatrixXd btpl = b.transpose() * pn * ln;
  Eigen::MatrixXd btpa = b.transpose() * pn * an;
  Eigen::MatrixXd atpb = an.transpose() * pn * b;
  Eigen::MatrixXd btpb2 = btpb - btpa * inv_atpa * atpb;
  Eigen::MatrixXd btpl2 = btpl - btpa * inv_atpa * atpl;

  PrintMatrix(btpb_, "btpb_");
  PrintMatrix(btpl_, "btpl_");
  PrintMatrix(btpb2, "btpb2");
  PrintMatrix(btpl2, "btpl2");

  Eigen::MatrixXd float_sd = SequentialLS(btpb2, btpl2);

  PrintMatrix(btpb_, "btpb_");
  PrintMatrix(btpl_, "btpl_");
  PrintMatrix(atpa, "atpa");
  PrintMatrix(inv_atpa, "atpa_inv");
  PrintMatrix(atpl, "atpl");
  PrintMatrix(btpa, "btpa");
  PrintMatrix(atpb, "atpb");
  PrintMatrix(float_sd, "float_sd");

  // solution based on sequential float ambiguities
  dx = inv_atpa * (atpl - atpb * float_sd);
  rover_coor->x += dx(0, 0);
  rover_coor->y += dx(1, 0);
  rover_coor->z += dx(2, 0);

  lm -= am * dx;
  ln -= an * dx + b * float_sd;

  vtpv = lm.transpose() * pm * lm + ln.transpose() * pn * ln;
  vtpv /= (ln.rows() + lm.rows() - num_states + 1);
  PrintMatrix(vtpv, "vtpv");
  PrintMatrix(inv_atpa, "inv_atpa");

  Eigen::MatrixXd atpa_mn_inv = inv_atpa;
  Eigen::MatrixXd test_new_std = atpa_mn_inv * vtpv(0, 0);
  PrintMatrix(test_new_std, "test_new_std");

  // float solution
  std_rtk_.x = test_new_std(0, 0);
  std_rtk_.y = test_new_std(1, 1);
  std_rtk_.z = test_new_std(2, 2);

  Eigen::MatrixXd ref_amb;
  Eigen::MatrixXd matrix_sd_2_dd = amb_tracker_.GetMatrixSD2DD(&ref_amb);

  Eigen::MatrixXd float_dd = matrix_sd_2_dd * float_sd - ref_amb;
  Eigen::MatrixXd float_dd_cvc =
      matrix_sd_2_dd * float_sd_cvc_ * matrix_sd_2_dd.transpose();
  PrintMatrix(matrix_sd_2_dd, "matrix_sd_2_dd");
  PrintMatrix(ref_amb, "ref_amb");
  PrintMatrix(float_dd, "float_dd");
  PrintMatrix(float_dd_cvc, "float_dd_cvc");

  Eigen::MatrixXd integer_dd = float_dd;
  double ratio = 0.0;
  double ratio_thres = 0.0;
  double adop = 0.0;
  Eigen::MatrixXd integer_sd;
  if (ref_amb.rows() != 0) {
    adop = pow(float_dd_cvc.determinant(), 1.0 / integer_dd.rows());
    ratio = ResolveAmbiguity(float_dd, float_dd_cvc, &integer_dd);
    if (!CheckLambda(ratio, adop)) {
      EncodeDetails("Invalid AR lambda:r=%.1f , adop=%.1f", ratio, adop);
      return false;
    }
    PrintMatrix(integer_dd, "integer_dd");
    PrintMatrix(integer_dd + ref_amb, "integer_dd + ref_amb");
    Eigen::MatrixXd matrix_dd_2_sd = matrix_sd_2_dd.transpose();

    integer_sd = matrix_dd_2_sd * (integer_dd + ref_amb);
    ratio_thres = mlambda_solver_.GetRatioThreshold(integer_dd.rows(),
                                                      real_time_ub_fail_rate_);
  } else {
    invalid_rtk_sub_infor_ = "New-arising set as reference";
    integer_sd = float_sd;
    ratio = 999.8888;
  }
  PrintMatrix(integer_sd, "integer_sd");
  PrintMatrix(float_sd, "float_sd");
  RestoreSDAmb(float_sd, &integer_sd);
  Eigen::MatrixXd diff_ar = float_sd - integer_sd;
  double diff_distance = diff_ar.norm();
  // update position after resolving integer-fixing
  Eigen::MatrixXd newdx = inv_atpa * atpb * (float_sd - integer_sd);

  Eigen::MatrixXd fake_ln = ln - (an * newdx + b * (integer_sd - float_sd));
  Eigen::MatrixXd fake_lm = lm - am * newdx;
  PrintMatrix(fake_lm, "fake_lm");
  PrintMatrix(fake_ln, "fake_ln");
  Eigen::MatrixXd vtpv_phase = fake_ln.transpose() * fake_ln;
  double std_phase_pure = sqrt(vtpv_phase(0, 0));
  double std_phase = sqrt(vtpv_phase(0, 0) / fake_ln.rows());

  vtpv =
      fake_lm.transpose() * pm * fake_lm + fake_ln.transpose() * pn * fake_ln;
  vtpv /= (fake_ln.rows() + fake_lm.rows() - num_states + 1);
  test_new_std = atpa_mn_inv * vtpv(0, 0);
  PrintMatrix(vtpv, "fake_vtpv");
  PrintMatrix(test_new_std, "fake_test_new_std");

  double fake_fix_std_v =
      sqrt(test_new_std(0, 0) + test_new_std(1, 1) + test_new_std(2, 2));
  _std_v = fake_fix_std_v;
  PrintMatrix(newdx, "newdx");
  auxiliary_infor_ = dx + newdx;

  if (!b_rtkfixed_) {
    if (FixJudgeCombined(ratio, ratio_thres, vtpv(0, 0), std_phase,
                      real_time_ub_fail_rate_)) {
      EncodeDetails(
          "ARFix:%3d%12.3f%12.3f%12.3f%12.3f%12.8f%12.3f%12.3f%12.3f\n",
          obs_rover.receiver_id(), obs_rover.gnss_second_s(), vtpv(0, 0), ratio,
          ratio_thres, real_time_ub_fail_rate_, std_phase, diff_distance,
          std_phase_pure);
      AddToFixedPool(integer_sd);
      ClearSequential();
      counter_for_low_ratio_ = 0;
      ln -= an * newdx + b * (integer_sd - float_sd);
      lm -= am * newdx;
      vtpv = lm.transpose() * pm * lm + ln.transpose() * pn * ln;
      vtpv /= (ln.rows() + lm.rows() - num_states + 1);
      PrintMatrix(vtpv, "vtpv");
      test_new_std = atpa_mn_inv * vtpv(0, 0);
      _std_v =
          sqrt(test_new_std(0, 0) + test_new_std(1, 1) + test_new_std(2, 2));
      std_rtk_.x = test_new_std(0, 0);
      std_rtk_.y = test_new_std(1, 1);
      std_rtk_.z = test_new_std(2, 2);
      rover_coor->x += newdx(0, 0);
      rover_coor->y += newdx(1, 0);
      rover_coor->z += newdx(2, 0);
    } else {
      ++counter_for_low_ratio_;
      if (_std_v > 0.25 || counter_for_low_ratio_ >= 20) {
        ResetFixedRtk();
      }
    }
  } else {
    ratio_ = BoundRatio(ratio);
    PointThreeDim denu;
    PointThreeDim dxyz;
    dxyz.x = newdx(0, 0);
    dxyz.y = newdx(1, 0);
    dxyz.z = newdx(2, 0);
    gnss_utility::dxyz2enu(*rover_coor, dxyz, &denu);
    double thres_hold = GetStdThresForNewSat(ratio, _std_v);
    PointThreeDim rover_coor_temp = *rover_coor;
    rover_coor_temp.x += newdx(0, 0);
    rover_coor_temp.y += newdx(1, 0);
    rover_coor_temp.z += newdx(2, 0);
    PointThreeDim last_fixed_rover(last_fixed_rover_pnt_.pos_x_m(),
                                   last_fixed_rover_pnt_.pos_y_m(),
                                   last_fixed_rover_pnt_.pos_z_m());
    PointThreeDim diff_delta = rover_coor_temp - last_fixed_rover;
    double dis_delta = gnss_utility::GetDistance(time_diff_delta_pos_,
                                                 diff_delta);
    if (CheckNewsatSolution(denu, thres_hold, dis_delta)) {
      EncodeDetails("NewSat: %12.3f %6.3f %6.3f\n", obs_rover.gnss_second_s(),
                     dis_delta, denu.Norm3D());
      new_fixed_phase_num_ = integer_sd.rows();
      AddToFixedPool(integer_sd);
      ClearSequential();
      counter_for_low_ratio_ = 0;
      // fix solution
      ln -= an * newdx + b * (integer_sd - float_sd);
      lm -= am * newdx;
      vtpv = lm.transpose() * pm * lm + ln.transpose() * pn * ln;
      vtpv /= (ln.rows() + lm.rows() - num_states + 1);
      PrintMatrix(vtpv, "vtpv");
      test_new_std = atpa_mn_inv * vtpv(0, 0);
      _std_v =
          sqrt(test_new_std(0, 0) + test_new_std(1, 1) + test_new_std(2, 2));
      std_rtk_.x = test_new_std(0, 0);
      std_rtk_.y = test_new_std(1, 1);
      std_rtk_.z = test_new_std(2, 2);
      *rover_coor = rover_coor_temp;
    } else {
      newdx.setZero();
      counter_for_low_ratio_ += 1;
      SeedToughSatGroup(ratio);
      ClearSequential();
      if (counter_for_low_ratio_ >= 10 && _std_v > 0.1) {
        ResetFixedRtk();
      }
    }
  }
  return true;
} // NOLINT

double GnssPntSolver::ResolveAmbiguity(const Eigen::MatrixXd& float_dd,
                                        const Eigen::MatrixXd& float_dd_cvc,
                                        Eigen::MatrixXd* inter_dd) {
  if (float_dd.cols() != 1) {
    return -1.0;
  }
  if (float_dd.rows() != float_dd_cvc.rows()) {
    return -1.0;
  }

  Eigen::MatrixXd inter_dd_temp = *inter_dd;
  mlambda_solver_.ResolveIntegerAmbiguity(float_dd, float_dd_cvc,
                                            &inter_dd_temp);
  inter_dd->col(0) = inter_dd_temp.col(0);

  real_time_ub_fail_rate_ = mlambda_solver_.GetUpperBoundFailureRate();
  ar_z_ = mlambda_solver_.GetArZ();
  ar_qzz_ = mlambda_solver_.GetArQzz();

  return mlambda_solver_.GetRatio();
}

bool GnssPntSolver::MotionUpdate(double time_sec, const PointThreeDim position,
                                  const double std_pos[3][3],
                                  const PointThreeDim velocity,
                                  const double std_vel[3][3]) {
  _predict_time_sec = time_sec;
  _predict_position = position;
  _predict_velocity = velocity;
  for (unsigned int i = 0; i < 3; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      _predict_std_pos[i][j] = std_pos[i][j];
      _predict_std_vel[i][j] = std_vel[i][j];
    }
  }
  return true;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
