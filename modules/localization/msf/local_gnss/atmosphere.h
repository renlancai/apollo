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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNS_ATMOSPHERE_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNS_ATMOSPHERE_H_

#include <vector>
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"

namespace apollo {
namespace localization {
namespace local_gnss {

class Atmosphere {
 public:
  Atmosphere(const PointThreeDim &rover_coor, const unsigned int gps_weeknum,
             const double gps_second_s);

  Atmosphere() {
    day_of_year_ = 0;
    for (unsigned int i = 0; i < 3; ++i) {
      pos_blh_[i] = 0.0;
    }
    ztd_dry_ = 0.0;
    ztd_wet_ = 0.0;
  }

  ~Atmosphere();

 public:
  // zenith troposphere delay
  void ZtdModel(double* ztd_dry, double* ztd_wet);

  double TropoDelay(const double elevation, const double azimuth);
  double TropoWet(const double elevation, const double azimuth);
  double TropoDry(const double elevation, const double azimuth);
  double GetMapWet(const double elevation, const double azimuth);
  double GetMapDry(const double elevation, const double azimuth);

 private:
  void ZtdMapNiel(const double elevation, const double azimuth,
                    double* map_wet, double* map_dry);

  double InterploateCoeff(const double coef[], double lat);

  double GetMapCoef(double el, double a, double b, double c);
  // GPS time to Day Of Year
  void Gpstime2Dof(const unsigned int gps_weeknum,
                   const double gps_week_second_s);

 private:
  double day_of_year_;
  double pos_blh_[3];
  double ztd_dry_;
  double ztd_wet_;
};

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
#endif
