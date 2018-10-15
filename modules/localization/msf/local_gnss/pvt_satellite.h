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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_PVT_SATELLITE_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_PVT_SATELLITE_H_

#include <vector>
#include <map>
#include "modules/localization/msf/local_gnss/leap_second.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/localization/msf/local_gnss/pvt_gnss.h"

namespace apollo {
namespace localization {
namespace local_gnss {

class SatelliteInterface {
 public:
  SatelliteInterface(const unsigned int max_size_eph_gps,
                     const unsigned int max_size_eph_bds,
                     const unsigned int max_size_eph_glo);

  SatelliteInterface();
  ~SatelliteInterface();

 public:
  bool SetEphMaxsize(const int max_size, GnssType sat_sys);
  bool SaveEphemeris(const GnssEphemeris& gnss_orbit);
  bool GetPosVelClock(const GnssType sat_sys, const int sat_prn,
                       const unsigned int gnss_week_num,
                       const double time_signal_transimited,
                       const double time_signal_duration,
                       PointThreeDim* position, PointThreeDim* velocity,
                       double* clk_bias, double* clk_drift, double* eph_toe);

  static double GetBandLength(const int prn,
                            const apollo::drivers::gnss::GnssBandID& fre_id,
                            int* glo_fre_num);
  static GnssType DetermineGnssType(
      const apollo::drivers::gnss::GnssBandID& band_id);

  inline double GetCurrentLeapSecond(unsigned int gps_week_num,
                                    double gps_week_second_s) {
    current_leap_second_s_ =
        global_leap_record_.GetLeapSecond(gps_week_num, gps_week_second_s);
    return current_leap_second_s_;
  }
  inline double GetCurrentLeapSecond() { return current_leap_second_s_; }

 private:
  bool CheckDuplicatedEph(const GnssEphemeris& raw_eph);

 private:
  std::map<EphKey, GnssEphemeris> map_gnss_eph_;
  GPSPvt gps_eph_;
  BeidouPvt bds_eph_;
  GlonassPvt glo_eph_;
  GalileoPvt gal_eph_;
  LeapSecond global_leap_record_;
  double current_leap_second_s_;
};

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
#endif
