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

#include "modules/localization/msf/local_gnss/atmosphere.h"
#include <math.h>
#include "modules/localization/msf/local_gnss/gnss_constants.hpp"

namespace apollo {
namespace localization {
namespace local_gnss {

Atmosphere::Atmosphere(const PointThreeDim& rover_coor,
                       const unsigned int gps_weeknum,
                       const double gps_second_s) {
  Gpstime2Dof(gps_weeknum, gps_second_s);
  gnss_utility::xyz2blh(rover_coor, &pos_blh_[0], &pos_blh_[1], &pos_blh_[2]);
  ZtdModel(&ztd_dry_, &ztd_wet_);
}

Atmosphere::~Atmosphere() {}

void Atmosphere::ZtdModel(double* ztd_dry, double* ztd_wet) {
  // too low or too high
  if (pos_blh_[2] < -100.0 || pos_blh_[2] > 1E4) {
    *ztd_dry = 0.0;
    *ztd_wet = 0.0;
    return;
  }
  const double zazel[] = {0.0, PI / 2.0};
  // temperature at sea level
  const double temp_sl = 15.0;
  double humi = 0.5;
  // standard atmosphere
  double hgt = pos_blh_[2] < 0.0 ? 0.0 : pos_blh_[2];
  double pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
  double temp = temp_sl - 6.5E-3 * hgt + 273.16;
  double e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));
  // saastamoninen model
  double z = PI / 2.0 - zazel[1];
  *ztd_dry = 0.0022768 * pres /
             (1.0 - 0.00266 * cos(2.0 * pos_blh_[0]) - 0.00028 * hgt / 1E3) /
             cos(z);
  *ztd_wet = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
  return;
}

void Atmosphere::ZtdMapNiel(const double elevation, const double azimuth,
                            double* map_wet, double* map_dry) {
  if (elevation <= 0.0) {
    *map_dry = 0.0;
    *map_wet = 0.0;
    return;
  }
  // hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75
  const double coef[][5] = {
      {1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
      {2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
      {62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},

      {0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
      {0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
      {0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},

      {5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
      {1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
      {4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}};
  // height correction
  const double aht[] = {2.53E-5, 5.49E-3, 1.14E-3};
  double elv = elevation;
  double lat = pos_blh_[0] * 180 / PI;
  double hgt = pos_blh_[2];
  // year from day_of_year 28, and add half a year for southern latitudes
  double y = (day_of_year_ - 28.0) / 365.25 + (lat < 0.0 ? 0.5 : 0.0);
  double cosy = cos(y * TWO_PI);
  lat = fabs(lat);
  double ah[3] = {0.};
  double aw[3] = {0.};
  for (int i = 0; i < 3; i++) {
    ah[i] = InterploateCoeff(coef[i], lat) -
            InterploateCoeff(coef[i + 3], lat) * cosy;
    aw[i] = InterploateCoeff(coef[i + 6], lat);
  }
  // ellipsoidal height is used instead of height above sea level
  double dm =
      (1.0 / sin(elv) - GetMapCoef(elv, aht[0], aht[1], aht[2])) * hgt / 1E3;
  *map_wet = GetMapCoef(elv, aw[0], aw[1], aw[2]);
  *map_dry = GetMapCoef(elv, ah[0], ah[1], ah[2]) + dm;
  return;
}

double Atmosphere::InterploateCoeff(const double coef[], double lat) {
  int i = static_cast<int>(lat / 15.0);
  if (i < 1) {
    return coef[0];
  } else if (i > 4) {
    return coef[4];
  }
  return coef[i - 1] * (1.0 - lat / 15.0 + i) + coef[i] * (lat / 15.0 - i);
}

double Atmosphere::GetMapCoef(double elv, double a, double b, double c) {
  double sinel = sin(elv);
  return (1.0 + a / (1.0 + b / (1.0 + c))) /
         (sinel + (a / (sinel + b / (sinel + c))));
}

void Atmosphere::Gpstime2Dof(const unsigned int gps_weeknum,
                             const double gps_week_second_s) {
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  double second_s = 0.0;
  apollo::localization::local_gnss::gnss_utility::GpsTime2DayTime(
      gps_weeknum, gps_week_second_s, &year, &month, &day, &hour, &minute,
      &second_s);
  day_of_year_ = apollo::localization::local_gnss::gnss_utility::GetDayOfYear(
      year, month, day);
}

double Atmosphere::TropoDelay(const double elevation, const double azimuth) {
  double map_wet = 0;
  double map_dry = 0;
  ZtdMapNiel(elevation, azimuth, &map_wet, &map_dry);
  return map_wet * ztd_wet_ + map_dry * ztd_dry_;
}

double Atmosphere::TropoWet(const double elevation, const double azimuth) {
  double map_wet = 0;
  double map_dry = 0;
  ZtdMapNiel(elevation, azimuth, &map_wet, &map_dry);
  return map_wet * ztd_wet_;
}

double Atmosphere::TropoDry(const double elevation, const double azimuth) {
  double map_wet = 0;
  double map_dry = 0;
  ZtdMapNiel(elevation, azimuth, &map_wet, &map_dry);
  return map_dry * ztd_dry_;
}

double Atmosphere::GestMapWet(const double elevation, const double azimuth) {
  double map_wet = 0;
  double map_dry = 0;
  ZtdMapNiel(elevation, azimuth, &map_wet, &map_dry);
  return map_wet;
}

double Atmosphere::GestMapDry(const double elevation, const double azimuth) {
  double map_wet = 0;
  double map_dry = 0;
  ZtdMapNiel(elevation, azimuth, &map_wet, &map_dry);
  return map_dry;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
