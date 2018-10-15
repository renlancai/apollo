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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_AMBIGUITY_TRACKER_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_AMBIGUITY_TRACKER_H_

#include <Eigen/Eigen>
#include <map>
#include <vector>
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"

namespace apollo {
namespace localization {
namespace local_gnss {

class AmbiguityTracker {
 public:
  AmbiguityTracker();
  ~AmbiguityTracker();
  typedef std::map<AmbKey, double> AmbLogger;

 public:
  // fixed amb operations
  inline bool SetFixedAmb(const apollo::drivers::gnss::GnssBandID &band_id,
                      const int &sat_prn, const double amb_val) {
    return SetLog(band_id, sat_prn, amb_val, &amb_fixed_);
  }

  inline bool SetFixedAmb(const apollo::localization::local_gnss::AmbKey &amb,
                      const double amb_val) {
    return SetLog(amb.band_id, amb.sat_prn, amb_val, &amb_fixed_);
  }

  bool GetFixedAmb(const apollo::drivers::gnss::GnssBandID &sat_band_id,
               const unsigned int sat_prn, double* ambiguity_val);

  inline bool GetFixedAmb(const apollo::localization::local_gnss::AmbKey &amb,
                      double* ambiguity_val) {
    return GetFixedAmb(amb.band_id, amb.sat_prn, ambiguity_val);
  }

  unsigned int GetNumFixedAmb();

  //  unfixed amb operations
  bool GetUnfixedAmbIndex(const GnssBandID &sat_band_id,
                       const unsigned int sat_prn, int* index);

  unsigned int GetNumUnfixedAmb();

  void ResetUnfixedPhase();

  bool GetUnfixedPhase(unsigned int index,
                     apollo::drivers::gnss::GnssBandID* band_id,
                     unsigned int* sat_prn);
  bool GetUnfixedPhase(unsigned int index, ObsKey* sat_obs);

  //  elevation
  bool GetElevation(const apollo::drivers::gnss::GnssBandID &sat_band_id,
                     const unsigned int sat_prn, double* elv_deg);

  double GetElevation(const AmbKey obs_key);

  bool SetElevation(const apollo::drivers::gnss::GnssBandID &sat_band_id,
                     const unsigned int sat_prn, const double &elv_deg);

  bool GetReferenceAmb(const AmbKey obs_key, double* val);

  //  deploy _amb_track to generate the transform matrix
  Eigen::MatrixXd GetMatrixSD2DD(Eigen::MatrixXd* ref_amb);

  bool UpdateSDAmb(const Eigen::MatrixXd &float_sd,
                     Eigen::MatrixXd* integer_sd);

  bool AddFixedAmb(const Eigen::MatrixXd &integer_sd);

  bool ReupdateReferenceAmb(
    const double sd_amb_res[apollo::drivers::gnss::GLO_G3]);

  bool DeleteDescendingAmb();
  bool DeleteSlipAmb(const apollo::drivers::gnss::GnssBandID &sat_band_id,
                       const unsigned int sat_prn);

  void ClearObsHistory();
  bool ClearAll();

  //  half cycle, not supported in apollo
  void ClearHalfCycleRecorder();
  void AddHalfCycleRecorder(const AmbKey &half_cycle_phase);
  bool IsHalpCycle(const AmbKey &half_cycle_phase);

 private:
  bool GetLog(const apollo::drivers::gnss::GnssBandID &sat_band_id,
               const unsigned int sat_prn, const AmbLogger &logger,
               double* val);
  bool SetLog(const apollo::drivers::gnss::GnssBandID &sat_band_id,
               const unsigned int sat_prn, const double &val,
               AmbLogger* logger);

  bool DeleteLog(const apollo::drivers::gnss::GnssBandID &sat_band_id,
                  const unsigned int sat_prn, AmbLogger* logger);
  int GetReferenceIndex(AmbKey obs);

  inline double Round(double x) {
    return static_cast<double>(std::floor(x + 0.5));
  }

 private:
  //  fixed ambiguities of all GNSS bands including reference satellite' band
  std::map<AmbKey, double> amb_fixed_;
  std::vector<AmbKey> unresolved_phase_;
  std::vector<AmbKey> reference_sat_;
  std::map<AmbKey, double> obs_elevation_deg_;
  std::vector<AmbKey> half_cycle_phase_;
};

}  //  namespace local_gnss
}  //  namespace localization
}  //  namespace apollo
#endif
