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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_UTILITY_HPP_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_UTILITY_HPP_

#include <math.h>
#include <stdarg.h>
#include <Eigen/Eigen>
#include <string>
#include <vector>
#include "modules/localization/msf/local_gnss/gnss_constants.hpp"

#include "modules/localization/proto/gnss_pnt_result.pb.h"
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"

namespace apollo {
namespace localization {
namespace local_gnss {

typedef apollo::drivers::gnss::GnssType GnssType;
typedef apollo::drivers::gnss::GnssBandID GnssBandID;
typedef apollo::drivers::gnss::EpochObservation EpochObservation;
typedef apollo::drivers::gnss::SatelliteObservation SatelliteObservation;
typedef apollo::drivers::gnss::BandObservation BandObservation;
typedef apollo::drivers::gnss::GlonassOrbit GlonassOrbit;
typedef apollo::drivers::gnss::GnssEphemeris GnssEphemeris;
typedef apollo::drivers::gnss::KepplerOrbit KepplerOrbit;

struct PointThreeDim {
  double x;
  double y;
  double z;
  PointThreeDim() {
    x = 0.0;
    y = 0.0;
    z = 0.0;
  }

  PointThreeDim(double t_x, double t_y, double t_z) {
    x = t_x;
    y = t_y;
    z = t_z;
  }

  PointThreeDim& operator=(const PointThreeDim& p2) {
    x = p2.x;
    y = p2.y;
    z = p2.z;
    return *this;
  }

  PointThreeDim& operator+=(const PointThreeDim& p2) {
    x = x + p2.x;
    y = y + p2.y;
    z = z + p2.z;
    return *this;
  }

  PointThreeDim operator+(const PointThreeDim& p2) {
    PointThreeDim t;
    t.x = this->x + p2.x;
    t.y = this->y + p2.y;
    t.z = this->z + p2.z;
    return t;
  }

  PointThreeDim operator-(const PointThreeDim& p2) {
    PointThreeDim t;
    t.x = this->x - p2.x;
    t.y = this->y - p2.y;
    t.z = this->z - p2.z;
    return t;
  }

  PointThreeDim SquareRoot() {
    PointThreeDim t;
    t.x = sqrt(x);
    t.y = sqrt(y);
    t.z = sqrt(z);
    return t;
  }

  PointThreeDim operator/(const double scale) {
    PointThreeDim t;
    t.x = x / scale;
    t.y = y / scale;
    t.z = z / scale;
    return t;
  }

  PointThreeDim operator*(const double scale) {
    PointThreeDim t;
    t.x = x * scale;
    t.y = y * scale;
    t.z = z * scale;
    return t;
  }

  double Norm3D() { return sqrt(x * x + y * y + z * z); }

  double GetDistance(const PointThreeDim& dest) {
    double dist = 0.0;
    dist += (x - dest.x) * (x - dest.x);
    dist += (y - dest.y) * (y - dest.y);
    dist += (z - dest.z) * (z - dest.z);
    return sqrt(dist);
  }
};

struct SatelliteInfor {
  short sat_prn; // NOLINT
  GnssType sat_sys;
  int index_in_obs;
  double clock_bias;
  double clock_drift;
  PointThreeDim position;
  PointThreeDim velocity;
  PointThreeDim direction;
  double distance;
  double elevation;
  double azimuth;
  bool multi_path;
  // week_num * SECOND_PER_WEEK + week_second_s
  double toe;
  unsigned int gnss_week;
  double time_signal_transmitted;
  double time_travles;
  SatelliteInfor() {
    sat_prn = 0;
    sat_sys = apollo::drivers::gnss::SYS_UNKNOWN;
    index_in_obs = -1;
    clock_bias = 0;
    clock_drift = 0;
    distance = 0;
    elevation = 0;
    azimuth = 0;
    multi_path = false;
    toe = -0.1;
    gnss_week = 0;
    time_signal_transmitted = 0.0;
    time_travles = 0.0;
  }
  bool IsSameSatellite(const SatelliteInfor& other) {
    return (sat_sys == other.sat_sys && sat_prn == other.sat_prn);
  }
  bool operator==(const SatelliteInfor& other) const {
    return (sat_sys == other.sat_sys && sat_prn == other.sat_prn);
  }
  void PrintSat(const bool& b_print = false) {
    if (!b_print) {
      return;
    }
    printf("%3d%3d%14.9f%14.3f%14.3f%14.3f%11.3f%11.3f%11.3f\n", sat_prn,
          static_cast<int>(sat_sys), clock_bias, position.x, position.y,
          position.z, velocity.x, velocity.y, velocity.z);
  }
};

struct ObsKey {
  apollo::drivers::gnss::GnssBandID band_id;
  unsigned int sat_prn;
  ObsKey(const apollo::drivers::gnss::GnssBandID& id, const unsigned int prn) {
    band_id = id;
    sat_prn = prn;
  }
  ObsKey() {
    band_id = apollo::drivers::gnss::GPS_L1;
    sat_prn = 0;
  }
  bool operator<(const ObsKey& key2) const {
    if (band_id < key2.band_id) {
      return true;
    }
    if (band_id == key2.band_id) {
      return sat_prn < key2.sat_prn;
    }
    return false;
  }
  bool operator==(const ObsKey& key2) const {
    return (band_id == key2.band_id) && (sat_prn == key2.sat_prn);
  }
  ObsKey& operator=(const ObsKey& key2) {
    band_id = key2.band_id;
    sat_prn = key2.sat_prn;
    return *this;
  }
};

typedef ObsKey AmbKey;

struct EphKey {
  GnssType gnss_type;
  unsigned int sat_prn;
  // toe = eph.toe + eph.week_num * sec_per_week
  double eph_toe;
  EphKey(const GnssType type, const unsigned int prn, double toe) {
    gnss_type = type;
    sat_prn = prn;
    eph_toe = toe;
  }
  EphKey(const GnssType type, const unsigned int prn,
         const unsigned int week_num, double toe) {
    gnss_type = type;
    sat_prn = prn;
    eph_toe = toe + week_num * SECOND_PER_WEEK;
  }
  EphKey() {
    gnss_type = apollo::drivers::gnss::SYS_UNKNOWN;
    sat_prn = 0;
    eph_toe = -0.1;
  }
  bool operator<(const EphKey& key2) const {
    if (gnss_type < key2.gnss_type) {
      return true;
    }
    if (gnss_type == key2.gnss_type) {
      if (sat_prn < key2.sat_prn) {
        return true;
      }
      if (sat_prn == key2.sat_prn) {
        return eph_toe < key2.eph_toe;
      }
      return false;
    }
    return false;
  }
  bool operator==(const EphKey& key2) const {
    return (gnss_type == key2.gnss_type) && (sat_prn == key2.sat_prn) &&
           (eph_toe == key2.eph_toe);
  }
  EphKey& operator=(const EphKey& key2) {
    gnss_type = key2.gnss_type;
    sat_prn = key2.sat_prn;
    eph_toe = key2.eph_toe;
    return *this;
  }
};

namespace gnss_utility {

inline int RoundDoubleToInt(double val) { return static_cast<int>(val); }

inline double sign(const double val) {
  if (val >= 0) {
    return 1.0;
  } else {
    return -1.0;
  }
}

inline double round(double x) {
  return static_cast<double>(std::floor(x + 0.5));
}

template <typename T>
int GetIndexInVector(const T& type, const std::vector<T>& group) {
  unsigned int size = group.size();
  for (unsigned int m = 0; m < size; m++) {
    if (type == group[m]) {
      return m;
    }
  }
  return -1;
}

template <typename T>
int AppendNewToVector(const T& type, const std::vector<T>& group) {
  if (GetIndexInVector<T>(type, group) == -1) {
    group.push_back(type);
    return 0;
  }
  return -1;
}

inline std::string FormatString(const char* format, ...) {
  const int buf_size = 2048;
  char temp[buf_size] = {'\0'};
  va_list args;
  va_start(args, format);
  vsprintf(temp, format, args);
  va_end(args);
  return std::string(temp);
}

inline void PrintPvtResult(const GnssPntResult& rover_pnt, double ratio) {
  std::string part1 = FormatString(
      "%6d%12.3f%4d%16.3f%16.3f%16.3f%4d%4.1f%6.1f%8.3f%8.3f%8.3f",
      rover_pnt.gnss_week(), rover_pnt.gnss_second_s(),
      static_cast<int>(rover_pnt.pnt_type()), rover_pnt.pos_x_m(),
      rover_pnt.pos_y_m(), rover_pnt.pos_z_m(), rover_pnt.sovled_sat_num(),
      rover_pnt.pdop(), ratio, rover_pnt.vel_x_m(), rover_pnt.vel_y_m(),
      rover_pnt.vel_z_m());
  std::string part2 =
      FormatString("%8.3f%8.3f%8.3f", rover_pnt.std_pos_x_m(),
                    rover_pnt.std_pos_y_m(), rover_pnt.std_pos_z_m());
  printf("%s%s\n", part1.c_str(), part2.c_str());
}

inline void PrintEigenMatrix(const Eigen::MatrixXd& t, const char* t_name,
                               const bool print_long = false) {
  if (t_name == NULL) {
    return;
  }
  printf("debug matrix %s row=%d col=%d\n", t_name, static_cast<int>(t.rows()),
         static_cast<int>(t.cols()));
  for (unsigned int r = 0; r < t.rows(); ++r) {
    for (unsigned int c = 0; c < t.cols(); ++c) {
      if (print_long) {
        printf("%40.20f", t(r, c));
      } else {
        printf("%16.8f", t(r, c));
      }
    }
    printf("\n");
  }
}

// time conversion
inline bool IsLeapYear(const unsigned int year) {
  if (year % 4 != 0) {
    return false;
  }
  if (year % 400 == 0) {
    return true;
  }
  if (year % 100 == 0) {
    return false;
  }
  /*if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0) {
    return true;
  }*/
  return true;
}

inline bool CheckDateForGnss(const int& year, const int& month,
                               const int& day) {
  if (year < 1981 || month < 1 || month > 12 || day < 1 || day > 31) {
    return false;
  }
  return true;
}

inline unsigned int GetDayOfYear(const int& year, const int& month,
                                const int& day) {
  const int dinmth[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int dayofy = 0;
  if (!CheckDateForGnss(year, month, day)) {
    return 0;
  }
  if (month == 1) {
    return day;
  }
  dayofy = 0;
  for (int m = 1; m <= (month - 1); ++m) {
    dayofy += dinmth[m];
    if (m == 2 && IsLeapYear(year)) {
      dayofy += 1;
    }
  }
  dayofy += day;
  return dayofy;
}

inline void GpsTime2DayTime(int weekno, double gpstime, int* year,
                                 int* month, int* day, int* hour, int* minute,
                                 double* second) {
  int dayofweek = RoundDoubleToInt(gpstime / 86400);
  double secofday = gpstime - dayofweek * 86400;
  *hour = RoundDoubleToInt(secofday / 3600);
  double sminute = secofday - *hour * 3600;
  *minute = RoundDoubleToInt(sminute / 60);
  *second = sminute - *minute * 60;

  int daysum = weekno * 7 + dayofweek;
  // 44244 is gps-time origin corrected julian-day
  double cor_julian_day = 44244 + daysum;

  double abs_julian_day = 1 + RoundDoubleToInt(cor_julian_day) + 2400000;
  double part_cor_jul = cor_julian_day - RoundDoubleToInt(cor_julian_day);
  int ih = RoundDoubleToInt((abs_julian_day - 1867216.25) / 36524.25);
  double tt2 = abs_julian_day + 1 + ih - RoundDoubleToInt(ih / 4);
  double tt3 = tt2 - 1720995;
  int th1 = RoundDoubleToInt((tt3 - 122.1) / 365.25);
  abs_julian_day = 365.25 * th1 -
                  (365.25 * th1 - RoundDoubleToInt(365.25 * th1));
  int ih2 = RoundDoubleToInt((tt3 - abs_julian_day) / 30.6001);
  *day = RoundDoubleToInt((tt3 - abs_julian_day -
        RoundDoubleToInt(30.6001 * ih2)) + part_cor_jul);
  *month = ih2 - 1;
  if (ih2 > 13) {
    *month = ih2 - 13;
  }
  *year = th1;
  if (*month <= 2) {
    *year = *year + 1;
  }
}

inline int DayTime2GpsTime(int year, int month, int day, int hour,
                                int minute, double second, double* gpstime) {
  int dayofw = 0;
  int dayofy = 0;
  int weekno = 0;
  *gpstime = -0.1;
  if (!CheckDateForGnss(year, month, day)) {
    return weekno;
  }
  // Convert day, month and year to day of year
  dayofy = GetDayOfYear(year, month, day);
  // Convert day of year and year into week numearth_ber and day of week
  int ttlday = 360;
  for (int yr = 1981; yr <= (year - 1); yr++) {
    ttlday += 365;
    if (IsLeapYear(yr)) {
      ttlday += 1;
    }
  }
  ttlday += dayofy;
  weekno = ttlday / 7;
  dayofw = ttlday - 7 * weekno;
  *gpstime = (hour * 3600 + minute * 60 + second + dayofw * 86400);
  return weekno;
}

// coordinate conversion
inline void llh2xyz(const double longitude, const double latitude,
                    const double height, double r_xyz[3]) {
  // earth parameters of WGS-84
  double a = 6378137;
  double f = 0.00335281066474;
  double b = a - a * f;
  double easquare = (a + b) * (a - b) / (a * a);
  double n = a / sqrt(1 - easquare * sin(latitude) * sin(latitude));

  r_xyz[0] = (n + height) * cos(latitude) * cos(longitude);
  r_xyz[1] = (n + height) * cos(latitude) * sin(longitude);
  r_xyz[2] = (n * (1 - easquare) + height) * sin(latitude);
}

inline void llh2xyz(const double longitude, const double latitude,
                    const double height, PointThreeDim* user) {
  double r_xyz[3] = {0.0};
  llh2xyz(longitude, latitude, height, r_xyz);
  user->x = r_xyz[0];
  user->y = r_xyz[1];
  user->z = r_xyz[2];
}

inline void xyz2llh(double* rr, double* lat, double* lon, double* rh) {
  // PNT solving done under WGS-84 with listed below
  const double earth_ae = 6378137;
  // const double earth_be = 6356752.3142;
  const double earth_fe = 1.0 / 298.257223563;
  double earth_e2 = 2 * earth_fe - earth_fe * earth_fe;

  double x = rr[0];
  double y = rr[1];
  double z = rr[2];

  if (fabs(y) < 1.0E-12) {
    if (x > 0.0) {
      *lon = 0.0;
    } else {
      *lon = PI;
    }
  } else {
    *lon = atan2(y, x);
  }
  double s = sqrt(x * x + y * y) + 1e-10;
  double zs = z / s;
  *rh = sqrt(x * x + y * y + z * z) - earth_ae;
  *lat = atan(zs / (1 - earth_e2 * earth_ae / (earth_ae + *rh + 1e-12)));

  int cnum = 0;
  int b_true = 1;
  do {
    cnum++;
    double nn1 = 0;
    nn1 = earth_ae / sqrt(1 - earth_e2 * pow(sin(*lat), 2));
    double lat1 = *lat;
    *rh = s / cos(*lat) - nn1;
    *lat = atan(zs / (1 - earth_e2 * nn1 / (nn1 + *rh + 1e-12)));
    if ((fabs(*lat - lat1) < 1e-12) || (cnum > 5)) {
      break;
    }
  } while (b_true);

  if (fabs(*lon) < 1e-12) {
    *lon = *lon + 2 * PI;
  }
}

inline void xyz2blh(const PointThreeDim& user, double* lat, double* lon,
                    double* alt) {
  double r[3] = {user.x, user.y, user.z};
  xyz2llh(r, lat, lon, alt);
}

inline void dxyz2enu(const double dxyz[3], const double lat, const double lon,
                     double denu[3]) {
  denu[0] = -sin(lon) * dxyz[0] + cos(lon) * dxyz[1];
  denu[1] = -sin(lat) * cos(lon) * dxyz[0] - sin(lat) * sin(lon) * dxyz[1] +
            cos(lat) * dxyz[2];
  denu[2] = cos(lat) * cos(lon) * dxyz[0] + cos(lat) * sin(lon) * dxyz[1] +
            sin(lat) * dxyz[2];
  return;
}

inline void enu2xyz(const double denu[3], const double lat, const double lon,
                    double dxyz[3]) {
  double matrix_r[3][3] = {-sin(lon),
                           cos(lon),
                           0,
                           -sin(lat) * cos(lon),
                           -sin(lat) * sin(lon),
                           cos(lat),
                           cos(lat) * cos(lon),
                           cos(lat) * sin(lon),
                           sin(lat)};
  // xyz = m.transpose() * denu
  for (unsigned int i = 0; i < 3; ++i) {
    dxyz[i] = 0;
    for (unsigned int j = 0; j < 3; ++j) {
      dxyz[i] += matrix_r[j][i] * denu[j];
    }
  }
  return;
}

inline void enu2xyz(const double denu[3], const double lat, const double lon,
                    PointThreeDim* xyz) {
  double dxyz[3] = {0.0};
  //
  enu2xyz(denu, lat, lon, dxyz);
  xyz->x = dxyz[0];
  xyz->y = dxyz[1];
  xyz->z = dxyz[2];
  return;
}

inline void dxyz2enu(const PointThreeDim& user, const PointThreeDim& dxyz,
                     PointThreeDim* denu) {
  double dxyz0[3] = {dxyz.x, dxyz.y, dxyz.z};
  double lat = 0;
  double lon = 0;
  double rh = 0;
  xyz2blh(user, &lat, &lon, &rh);
  double denu0[3] = {0.0};
  dxyz2enu(dxyz0, lat, lon, denu0);
  denu->x = denu0[0];
  denu->y = denu0[1];
  denu->z = denu0[2];
}

inline double GetDistance(const PointThreeDim& src, const PointThreeDim& dest) {
  double dist = 0.0;
  dist += (src.x - dest.x) * (src.x - dest.x);
  dist += (src.y - dest.y) * (src.y - dest.y);
  dist += (src.z - dest.z) * (src.z - dest.z);
  return sqrt(dist);
}

inline double GetCosineDirection(const PointThreeDim& src,
                               const PointThreeDim& dest,
                               PointThreeDim* cosine) {
  double dist = GetDistance(src, dest);
  cosine->x = (src.x - dest.x) / dist;
  cosine->y = (src.y - dest.y) / dist;
  cosine->z = (src.z - dest.z) / dist;
  return dist;
}

inline void GetEleAzm(const PointThreeDim& user, const PointThreeDim& sat,
                    double* elv_deg, double* azm_deg) {
  double lat = 0;
  double lon = 0;
  double alt = 0;

  xyz2blh(user, &lat, &lon, &alt);
  double dx = sat.x - user.x;
  double dy = sat.y - user.y;
  double dz = sat.z - user.z;

  double xp =
      -sin(lat) * cos(lon) * dx - sin(lat) * sin(lon) * dy + cos(lat) * dz;
  double yp = -sin(lon) * dx + cos(lon) * dy;
  double zp =
      cos(lat) * cos(lon) * dx + cos(lat) * sin(lon) * dy + sin(lat) * dz;
  *elv_deg = atan(zp / sqrt(xp * xp + yp * yp)) / PI * 180;
  if (fabs(yp) < 1.0E-12) {
    if (xp > 0.0) {
      *azm_deg = 0.0;
    } else {
      *azm_deg = PI;
    }
  } else {
    *azm_deg = atan2(yp, xp);
  }
  *azm_deg = *azm_deg / PI * 180;
  return;
}

inline Eigen::MatrixXd DcmEcef2Navi(const PointThreeDim& pos) {
  // dcm from ecef XYZ to local navigation
  double lat = 0;
  double lon = 0;
  double rh = 0;
  gnss_utility::xyz2blh(pos, &lat, &lon, &rh);
  Eigen::Matrix3d matrix_r;
  matrix_r(0, 0) = -sin(lon);
  matrix_r(0, 1) = cos(lon);
  matrix_r(0, 2) = 0;
  matrix_r(1, 0) = -sin(lat) * cos(lon);
  matrix_r(1, 1) = -sin(lat) * sin(lon);
  matrix_r(1, 2) = cos(lat);
  matrix_r(2, 0) = cos(lat) * cos(lon);
  matrix_r(2, 1) = cos(lat) * sin(lon);
  matrix_r(2, 2) = sin(lat);
  return matrix_r;
}

inline Eigen::Matrix3d DcmBody2Navi(const double pitch, const double roll,
                               const double yaw) {
  // keep following codes for further non-Eigen environment
  // NOTICE: positive yaw being North by West (= anti-clockwise).
  double matrix_body2navi[3][3] = {0.0};
  matrix_body2navi[0][0] =
      cos(roll) * cos(yaw) - sin(pitch) * sin(yaw) * sin(roll);
  matrix_body2navi[0][1] = -cos(pitch) * sin(yaw);
  matrix_body2navi[0][2] =
      sin(roll) * cos(yaw) + cos(roll) * sin(pitch) * sin(yaw);
  matrix_body2navi[1][0] =
      cos(roll) * sin(yaw) + sin(roll) * sin(pitch) * cos(yaw);
  matrix_body2navi[1][1] = cos(pitch) * cos(yaw);
  matrix_body2navi[1][2] =
      sin(roll) * sin(yaw) - cos(roll) * sin(pitch) * cos(yaw);
  matrix_body2navi[2][0] = -sin(roll) * cos(pitch);
  matrix_body2navi[2][1] = sin(pitch);
  matrix_body2navi[2][2] = cos(roll) * cos(pitch);

  Eigen::Matrix3d temp;
  for (unsigned int i = 0; i < 3; ++i) {
    for (unsigned int j = 0; j < 3; ++j) {
      temp(i, j) = matrix_body2navi[i][j];
    }
  }
  return temp;
}

inline Eigen::MatrixXd RoundMatrix(const Eigen::MatrixXd& m1) {
  Eigen::MatrixXd m2 = m1;
  for (unsigned int r = 0; r < m2.rows(); ++r) {
    for (unsigned int c = 0; c < m2.cols(); ++c) {
      m2(r, c) = static_cast<int>(m2(r, c) + 0.5 * sign(m2(r, c)));
    }
  }
  return m2;
}

}  // namespace gnss_utility

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo

#endif
