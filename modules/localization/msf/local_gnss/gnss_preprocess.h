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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_PREPROCESS_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_PREPROCESS_H_
#include <Eigen/Eigen>
#include <map>
#include <vector>
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/localization/msf/local_gnss/pvt_satellite.h"

namespace apollo {
namespace localization {
namespace local_gnss {

typedef apollo::localization::local_gnss::ObsKey SlipKey;

class PreProcessor {
 public:
  PreProcessor();
  ~PreProcessor();

  bool SetResolvedBands(
      const std::vector<apollo::drivers::gnss::GnssBandID> &bands);

  void SetGlobalEphemerisPtr(SatelliteInterface *ptr_eph);

  bool DetectBasedOnGeometry(const std::vector<GnssType> &related_gnss_type,
                             const PointThreeDim &pos,
                             const EpochObservation &newobs,
                             const std::vector<SatelliteInfor> &sat_list);

  // not supported yet
  bool DetectGeometryFree(EpochObservation* newobs);

  // return true and set slip_val when slip exists
  bool CheckSlipValue(const apollo::drivers::gnss::GnssBandID &band_id,
                  const unsigned int sat_prn, double* slip_val);

  unsigned int GetSlipSize();

  PointThreeDim GetVelocity();

  PointThreeDim GetVelocityStd();

  PointThreeDim GetDeltaPos();

  PointThreeDim GetPositionStd();

  void SetPresiceCoor(const double upt_time, const PointThreeDim &rtk_result,
                   const PointThreeDim &std_result);

  bool IsPreciseSet();

  bool ReInitialize();

  void EnablePrint(const bool &d_print);

 private:
  void Initialize();

  void Reset();

  void PrintMatrix(const Eigen::MatrixXd &t, const char *t_name);

  int CheckBandPhaseObs(const BandObservation &band_obs1,
                           const BandObservation &band_obs2);

  bool UpdateSatInforWithNewEph(SatelliteInfor* last_sat);

  bool SaveLastEpochInfor(
      const PointThreeDim &pos,
      const EpochObservation &newobs,
      const std::vector<SatelliteInfor> &sat_list);

  bool CalculateAverageVel(const PointThreeDim &pos,
                        const EpochObservation &newobs,
                        PointThreeDim* pos_corr);

 private:
  bool debug_print_;
  unsigned int user_id_;
  double phase_precision_;
  PointThreeDim velocity_enu_;
  PointThreeDim std_vel_;
  PointThreeDim delta_pos_;

  SatelliteInterface *global_ephemeris_ptr_;
  // set band to be soved
  std::vector<apollo::drivers::gnss::GnssBandID> band_to_solve_;
  std::vector<GnssType> gnss_to_solve_;

  std::map<SlipKey, double> slip_recorder_;

  PointThreeDim last_pos_;
  PointThreeDim last_pos_std_;
  EpochObservation last_obs_;
  std::vector<apollo::localization::local_gnss::SatelliteInfor> last_sat_list_;

  double last_obs_time_;
  bool precise_set_coor_;
  double delta_time_;
};

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
#endif
