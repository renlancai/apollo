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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_DEBUG_TOOLS_HPP_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_DEBUG_TOOLS_HPP_

#include <Eigen/Eigen>
#include <vector>
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"

namespace apollo {
namespace localization {
namespace local_gnss {

inline bool WriteObs(const char* file, const EpochObservation& user_obs,
                      std::vector<SatelliteInfor> user_sat_vector_used) {
  if (file == NULL) {
    return false;
  }
  FILE* fp = fopen(file, "a+");
  if (fp == NULL) {
    return false;
  }
  fprintf(fp, "%3d%16.3f%4d", user_obs.receiver_id(), user_obs.gnss_second_s(),
          user_obs.sat_obs_num());
  if (user_obs.has_position_x() && user_obs.has_position_x() &&
      user_obs.has_position_x()) {
    fprintf(fp, "%16.3f%16.3f%16.3f", user_obs.position_x(),
            user_obs.position_y(), user_obs.position_z());
  }
  fprintf(fp, "\n");
  int size = user_sat_vector_used.size();
  for (int i = 0; i < size; ++i) {
    SatelliteInfor temp = user_sat_vector_used[i];
    fprintf(fp, "%3d%3d%16.3f%20.6f%12.5f%16.3f%16.3f%16.3f%20.3f\n",
            temp.sat_prn, temp.sat_sys,
            user_obs.sat_obs(temp.index_in_obs).band_obs(0).pseudo_range(),
            temp.time_signal_transmitted, temp.time_travles, temp.position.x,
            temp.position.y, temp.position.z, temp.toe);
  }
  fclose(fp);
  return true;
}

inline void WritePnt(FILE** fp, const unsigned int gnss_week_num,
                      const double gnss_week_second_s,
                      const PointThreeDim& denu, const double bs_len,
                      const unsigned int num_valid_sat, const double ratio,
                      const PointThreeDim& rover,
                      const Eigen::MatrixXd& auxiliary_infor,
                      const double baser_gnss_week_seconds,
                      const double std_rover, const double std_baser,
                      const double curr_leap, const PointThreeDim vel_rover,
                      const double std_x, const double std_y,
                      const double std_z, const int pnt_type) {
  if (*fp == NULL) {
    return;
  }
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  double second_s = 0;
  apollo::localization::local_gnss::gnss_utility::GpsTime2DayTime(
      gnss_week_num, gnss_week_second_s, &year, &month, &day, &hour, &minute,
      &second_s);
  fprintf(*fp,
          "%12.3f%12.3f%12.3f%12.3f%12.3f%4d%10.1f%5d %02d %02d %02d "
          "%02d%5.1f%14.3f%14.3f%14.3f",
          gnss_week_second_s, denu.x, denu.y, denu.z, bs_len, num_valid_sat,
          ratio, year, month, day, hour, minute, second_s, rover.x, rover.y,
          rover.z);
  fprintf(*fp, "%8.3f%8.3f%8.3f", vel_rover.x, vel_rover.y, vel_rover.z);
  // zwd + relative clock drift + glonass-IFB
  for (unsigned int i = 3; i < auxiliary_infor.rows(); ++i) {
    fprintf(*fp, "%10.3f", auxiliary_infor(i, 0));
  }
  // for real time debug, print baser.time
  fprintf(*fp, "%12.2f", baser_gnss_week_seconds);
  fprintf(*fp, "%7.3f", std_rover);
  fprintf(*fp, "%7.3f", std_baser);
  fprintf(*fp, "  %6.3f%6.3f%6.3f%4d", std_x, std_y, std_z, pnt_type);
  fprintf(*fp, "\n");
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo

#endif
