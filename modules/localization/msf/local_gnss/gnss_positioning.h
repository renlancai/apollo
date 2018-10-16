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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_POSITIONING_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_POSITIONING_H_

#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);               \
  TypeName& operator=(const TypeName&)

#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <string>

#include "modules/localization/proto/gnss_pnt_result.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"

#include "modules/localization/msf/local_gnss/ambiguity_tracker.h"
#include "modules/localization/msf/local_gnss/ar_mlambda.h"
#include "modules/localization/msf/local_gnss/atmosphere.h"
#include "modules/localization/msf/local_gnss/gnss_preprocess.h"
#include "modules/localization/msf/local_gnss/pvt_satellite.h"

namespace apollo {
namespace localization {
namespace local_gnss {

enum PositionOption {
  STANDARD = 0,
  TIME_DIFF_PHASE = 1,
  CODE_DIFF = 2,
  RTK = 3
};

enum TropsphereOption {
  ZTD_CORRECT = 0,
  ZTD_ESTIMATE = 1
};
enum IonosphereOption {
  IONO_OFF = 0,
  IONO_CORRECT = 1,
  IONO_ESTIMATE_VTEC = 2,
  IONO_ESTIMATE_STEC = 3
};

enum ReturnCode {
  RET_BAD_SPP = -1,
  RET_SPP_MODE = 0,
  RET_TIME_DIFF_PHASE_MODE = 1,
  RET_NO_VALID_BASER = 2,
  RET_BIG_BASE_DATA_GAP = 3,
  RET_INVALID_BASE_COORDINATE = 4,
  RET_INVALID_BASE_SPP = 5,
  RET_NOT_ENOUGH_COMMON_SAT = 6,
  RET_CODE_DIFF_MODE = 7,
  RET_BIG_BASE_DATA_GAP_RTK = 8,
  RET_INVALID_RTK = 9,
  RET_VALID_RTK = 10
};

class SensorPntSolver {
 public:
  SensorPntSolver() {}
  virtual ~SensorPntSolver() {}
  virtual bool CaculateHeading(const PointThreeDim& ant_base,
                              const PointThreeDim& ant_slave,
                              const PointThreeDim& phase_std) = 0;

  virtual int UpdateBaseCoor(const EpochObservation& rover_obs,
                               const EpochObservation& baser_obs) = 0;
};

class GnssPntSolver : public SensorPntSolver {
 public:
  GnssPntSolver();
  ~GnssPntSolver();

  // stratgy interface
  bool SetPositionOption(
      const apollo::localization::local_gnss::PositionOption position_option);
  PositionOption GetPositionOption();
  bool SetTropsphereOption(
      const apollo::localization::local_gnss::TropsphereOption trop_option);
  bool SetIonosphereOption(
      const apollo::localization::local_gnss::IonosphereOption iono_option);

  bool SetElevationCutoffDeg(const double elevation_cutoff);
  double GetElevationCutoffDeg();

  void SetBaserBufferSize(const unsigned int baser_buffer_size);
  int GetBaserBufferSize();

  void SetMaxGpsEphSize(const unsigned int max_gps_eph_size);
  void SetMaxBdsEphSize(const unsigned int max_bds_eph_size);
  void SetMaxGloEphSize(const unsigned int max_glo_eph_size);

  void SetMaxGnssSystem(const unsigned int max_gnss_sys_num);

  void EnableHalfCycleAr(const bool b_enable);
  bool GetHalfCycleAr();

  void EnanleCycleSlipFix();
  bool GetCycleSlipFix();

  void SetGlonassIfb(double ifb);
  double GetGlonassIfb();

  void SetBaserCoordinate(const PointThreeDim& baser_coor);
  void SetBaserCoordinate(double x, double y, double z);
  PointThreeDim GetBaserCoor();

  void SetEnableExternalPrediction(bool b_enable);
  bool GetEnableExternalPrediction();

  void SetEnableTdPhaseUpdate(bool b_enable);
  bool GetEnableTdPhaseUpdate();

  void SetDebugPrint(const bool& d_print);

  void SetSynchTimeGapThreshold(
      const double& synch_time_gap_threshold = 1.0);
  double GetSynchTimeGapThreshold();

  // only for debugging to log pvt and fixed ambiguities to buffer files
  bool SetRtkResultFile(char* rtk_result_file);
  bool SetAmbiguityFile(char* ambiguity_recorder_file);

  // core interface for solving gnss
  bool SaveGnssEphemris(
      const GnssEphemeris& gnss_orbit);
  bool SaveBaserObservation(const EpochObservation& baser_obs);
  int Solve(const EpochObservation& rover_obs,
            GnssPntResult* rover_pnt);
  int SolveWithBaser(const EpochObservation& rover_obs,
                       const EpochObservation& baser_obs,
                       const bool& valid_base,
                       GnssPntResult* rover_pnt);

  // interface to get running status
  bool GetFixStatus();
  void ClearFixedRtk();
  double GetLeapSecond(unsigned int gps_week_num, double gps_week_second_s);
  double GetRatio();
  std::string GetInvalidRtkDetail();

  // interface to seed other sensors information
  bool MotionUpdate(double time_sec, const PointThreeDim position,
                     const double std_pos[3][3], const PointThreeDim velocity,
                     const double std_vel[3][3]);

 private:
  void Initialize();
  bool CheckBaserObs(double std, double distance = 0.0);

  void PrintMatrix(const Eigen::MatrixXd& t, const char* t_name);

  double RoundDouble(double x);

  void AddSolvedBand(const GnssBandID& b_id);

  int EnableGlonassIfbEstimated();

  bool IsGlonassExisted(const std::vector<GnssType>& related_gnss_type);

  double WeightScaleOnGnssSystem(const GnssType& type);

  inline void ResetFixedRtk() {
    ClearSequential();
    b_rtkfixed_ = false;
    amb_tracker_.ClearAll();
    ratio_ = 0.0;
    ratio_epoch_num_ = 0;
    counter_for_low_ratio_ = 0;
    tough_fixing_pool_.clear();
  }

  double GetBandLength(const int prn,
                     const apollo::drivers::gnss::GnssBandID& fre_id,
                     int* glo_fre_num);

  void LogFixedAmbToBuffer(const char* amb_log);

  void CheckConsecutiveTimeOffset(unsigned int week_num, double week_sec);

  bool IsEnabledCodeDiff(const double& data_outage,
                        const double& dis_rover2baser);

  bool IsEnabledPhaseDiff(const double& data_outage,
                         const double& dis_rover2baser);

  int UpdateBaseCoor(const EpochObservation& rover_obs,
                       const EpochObservation& baser_obs);

  bool CaculateHeading(const PointThreeDim& ant_base,
                      const PointThreeDim& ant_slave,
                      const PointThreeDim& phase_std);

  void EncodeDetails(const char* format, ...);

 private:
  bool CheckPossibleHalfSlip(const unsigned int loss_lock_index);

  bool ExcludeSemiCycleSat(const SatelliteObservation& sat_obs);

  bool CheckSatObsForSPP(const SatelliteObservation& sat_obs);
  // step 1
  double GetStandardPosition(const EpochObservation obs,
                           std::vector<SatelliteInfor>* sate_vector_used,
                           std::vector<GnssType>* related_gnss_type,
                           PointThreeDim* spp_result,
                           PointThreeDim* spp_dop);

  double GetStandardVelocity(const EpochObservation& obs,
                           const std::vector<SatelliteInfor>& sate_vector_used,
                           const std::vector<GnssType>& related_gnss_type,
                           PointThreeDim* spv_result);

  bool DeleteElvCutoffSat(
      const double ele_cutoff, std::vector<SatelliteInfor>* sate_vector_used);

  // synchronize
  bool SynchronizeBaser(const unsigned int gnss_week_num,
                         const double gnss_week_second_s,
                         EpochObservation* baser_obs,
                         const double& gap_threshold = 1.0);

  // step 2
  unsigned int SelectCommonSatellite(
      const std::vector<SatelliteInfor> sate_used_rover,
      const std::vector<SatelliteInfor> sate_used_baser,
      const PointThreeDim& baser_coor,
      std::vector<SatelliteInfor>* common_in_rover,
      std::vector<SatelliteInfor>* common_in_baser,
      std::vector<GnssType>* related_gnss_type);

  // step 3
  // TO DO(Renlan Move the baser sat update to select function):
  bool GetCodeDiffPosition(const EpochObservation& obs_rover,
                          const EpochObservation& obs_baser,
                          const std::vector<SatelliteInfor>& common_in_baser,
                          const std::vector<GnssType>& related_gnss_type,
                          std::vector<SatelliteInfor>* common_in_rover,
                          PointThreeDim* code_dif_result);
  // step 4
  // construct single difference observation equation
  bool GetGnssSystemToSolve();

  bool FilterOutGnssSystem(std::vector<GnssType>* related_gnss_type);

  bool CheckSlip(const apollo::drivers::gnss::GnssBandID& band_id,
                  const unsigned int sat_prn,
                  const unsigned int loss_lock_index_rover,
                  const unsigned int loss_lock_index_baser);

  bool IsToughNewSat(const ObsKey& new_sat_obs);
  bool SeedToughNewSat(const ObsKey& new_sat_obs, double ratio);
  bool SeedToughSatGroup(double ratio);

  bool CheckSlipOfUnfixedPhase();

  bool GetPhaseDiffPosition(const EpochObservation& obs_rover,
                           const EpochObservation& obs_baser,
                           const std::vector<SatelliteInfor>& common_in_rover,
                           const std::vector<SatelliteInfor>& common_in_baser,
                           const std::vector<GnssType>& related_gnss_type,
                           const PointThreeDim& baser_vel,
                           PointThreeDim* phase_diff_result);

  // quality validation block
  bool IsSafeStandardPointPos(const double std_rover, const double pdop);

  bool IsSafeCodeDiffPos(const double dis, const double std_dif,
                          const bool valid_delta_pos);

  bool IsSafeFixedSolution(const unsigned int gnss_type_size,
                           const unsigned int num_fixed_phase);

  int IsValidBandObs(const BandObservation& band_obs1,
                     const BandObservation& band_obs2);

  bool GetFixedAmb(const apollo::drivers::gnss::GnssBandID& band_id,
                 const unsigned int sat_prn, double* amb_val, int* index_unfix);

  bool UpdateBandElevation(const apollo::drivers::gnss::GnssBandID& band_id,
                        const unsigned int sat_prn, const double& elv);
  // Sequential Least Square
  Eigen::MatrixXd SequentialLS(const Eigen::MatrixXd& btpb,
                             const Eigen::MatrixXd& btpl);

  bool RestoreSDAmb(const Eigen::MatrixXd& float_sd,
                      Eigen::MatrixXd* integer_sd);

  bool AddToFixedPool(const Eigen::MatrixXd& integer_sd);
  bool ClearSequential();

  bool FixJudge(double ratio, const double threshold, double vtpv,
                 double std_phase, const unsigned int valid_num = 2);

  bool FixJudgeCombined(double ratio, const double threshold, double vtpv,
                     double std_phase, double fail_rate = 0.01);

  bool IsEnoughPhaseObs(const unsigned int valid_phase_size,
                         const unsigned int raw_gnss_type_size,
                         const unsigned int valid_gnss_type_size);

  bool ConstructSDEquation(const EpochObservation& obs_rover,
                             const EpochObservation& obs_baser,
                             const std::vector<SatelliteInfor>& common_in_rover,
                             const std::vector<SatelliteInfor>& common_in_baser,
                             const std::vector<GnssType>& related_gnss_type,
                             const PointThreeDim& baser_coor,
                             PointThreeDim* rover_coor);

  double GetStdThresForNewSat(const double new_ratio, const double std_v);

  void WeightOnPostResidual(const double std_v,
                               const std::vector<unsigned int>& range_type,
                               const Eigen::MatrixXd& post_res,
                               Eigen::MatrixXd* px_temp);

  void RemoveAmbWithAbnormalRes(const std::vector<ObsKey>& phase_recorder,
                    const double& std_v,
                    const std::vector<unsigned int>& range_type,
                    const Eigen::MatrixXd& post_res,
                    const double band_res_sum[apollo::drivers::gnss::GLO_G3]);

  void BoundRangePrecision(const double std_dif);

  PointThreeDim BoundedRtkStd();

  double ResolveAmbiguity(const Eigen::MatrixXd& float_dd,
                           const Eigen::MatrixXd& float_dd_cvc,
                           Eigen::MatrixXd* inter_dd);

  bool CheckLambda(const double ratio, const double adop);

  bool IsAbnormalFixedSolutionWithNewPhase(
      const bool& is_fixed, const double std_fixed,
      const unsigned int unknown_phase_num);

  bool IsNeedRecursion(const bool& is_fixed, const double std_fixed,
                      const unsigned int unknown_phase_num,
                      const unsigned int common_phase_num);

  double BoundRatio(const double& ratio);

  bool CheckNewsatSolution(const PointThreeDim& denu, const double& thres,
                             const double& dis_time_diff = 0.0);

 private:
  PositionOption position_option_;
  TropsphereOption ztd_option_;
  IonosphereOption iono_option_;
  double elevation_cutoff_deg_;

  double range_precision_;
  double phase_precision_;
  unsigned int max_gnss_sys_num_;

  double ratio_;
  unsigned int ratio_epoch_num_;
  unsigned int num_valid_sat_;
  double _std_v;
  unsigned int gnss_weeknum_;
  double gnss_week_second_s_;

  apollo::localization::local_gnss::PointThreeDim baser_coor_;
  apollo::localization::local_gnss::PointThreeDim baser_vel_;

  apollo::localization::local_gnss::PointThreeDim rover_coor_;

  unsigned int max_size_baser_buffer_;
  std::vector<EpochObservation> baser_obs_;
  unsigned int max_size_rover_buffer_;
  std::vector<EpochObservation> rover_obs_;

  SatelliteInterface global_ephemeris_;
  // set band to be soved
  std::vector<apollo::drivers::gnss::GnssBandID> band_to_solve_;
  std::vector<GnssType> gnss_to_solve_;

  // ambiguity tracking
  AmbiguityTracker amb_tracker_;

  // fix indicator
  bool b_rtkfixed_ = false;
  PointThreeDim std_rtk_;
  PointThreeDim std_rtk_thres_of_phase_dop_;
  PointThreeDim rtk_dop_phase_;
  PointThreeDim std_code_diff_;
  PointThreeDim std_doppler_diff_;
  // sequential ambiguity information
  Eigen::MatrixXd btpb_;
  Eigen::MatrixXd btpl_;
  Eigen::MatrixXd float_sd_cvc_;

  Eigen::MatrixXd auxiliary_infor_;

  // ar = ambiguity resolution
  bool enable_half_cycle_ar_;
  bool enable_cycle_slip_fix_;

  apollo::localization::local_gnss::PreProcessor rover_preprocessor_;
  apollo::localization::local_gnss::PreProcessor baser_preprocessor_;

  // Recall for a prior value, depending on receiver type
  double glonass_ifb_;

  // fixed baser
  bool external_fix_baser_;

  // sins-ekf/lidar/svo prediction
  bool enable_external_prediction_;
  double _predict_time_sec;
  PointThreeDim _predict_position;
  double _predict_std_pos[3][3];
  PointThreeDim _predict_velocity;
  double _predict_std_vel[3][3];

  // time-difference phase update
  bool enable_td_phase_update_;

  unsigned int counter_for_low_ratio_;
  unsigned int num_sd_phase_recursive_;
  //
  char* file_rtk_result_;
  FILE* fp_rtk_result_;
  char* file_amb_log_;

  // add a pool to monitor new satellite amb-fix states
  std::map<ObsKey, double> tough_fixing_pool_;
  unsigned int max_sat_num_baser_;

  // based on _p_failure_ILS and number of DD amb
  double real_time_ub_fail_rate_;
  ARMLambda mlambda_solver_;

  Eigen::MatrixXd ar_z_;
  Eigen::MatrixXd ar_qzz_;

  std::string invalid_rtk_sub_infor_;

  // distance from rover to base
  double distance_rover2baser_;

  double synch_time_gap_threshold_ = 1.0;
  unsigned int good_ratio_ = 0;
  bool valid_time_diff_ = false;
  PointThreeDim time_diff_delta_pos_;
  GnssPntResult last_fixed_rover_pnt_;
  unsigned int new_fixed_phase_num_ = 0;

  unsigned int _continuous_risky_num = 0;

  bool test_ins_aid_ = false;
  bool debug_print_ = false;
  DISALLOW_COPY_AND_ASSIGN(GnssPntSolver);
};

class GnssDualAntSolver : public GnssPntSolver {
 public:
  GnssDualAntSolver() { SetSynchTimeGapThreshold(0.001); }
  void SetDualAntennaMode(bool dual_antenna_mode);

  bool SetDualAntLeverArm(const PointThreeDim& ant_primary,
                              const PointThreeDim& ant_secondary);

  int SolveHeading(const EpochObservation& rover_obs,
                    GnssPntResult* rover_pnt,
                    double* heading, double* hd_std,
                    bool* valid_heading);

  int SolveHeadingWithBaser(const EpochObservation& rover_obs,
                               const EpochObservation& baser_obs,
                               GnssPntResult* rover_pnt,
                               double* heading, double* hd_std,
                               bool* valid_heading);

 private:
  double CorrectYaw(const double& yaw);
  bool CaculateHeading(const PointThreeDim& ant_base,
                      const PointThreeDim& ant_slave,
                      const PointThreeDim& phase_std);

  bool GetHeadingWithDualAntenna(double* heading, double* hd_std);
  int UpdateBaseCoor(const EpochObservation& rover_obs,
                       const EpochObservation& baser_obs);

 private:
  // dual antenna baseline length constrain
  double dual_ant_baseline_len_ = 0.0;
  PointThreeDim relative_lever_arm_;
  double rotation_dual_ant_ = 0.0;
  double heading_dual_ant_ = 0.0;
  double std_heading_ = 0.0;
  const double max_heading_std_ = 5.0;  // degree
  bool dual_antenna_mode_ = true;
  bool strict_synch_ = false;
};

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo

#endif
