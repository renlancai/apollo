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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_PVT_GNSS_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_PVT_GNSS_H_
#include <math.h>
#include <Eigen/Eigen>
#include <vector>
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/localization/msf/local_gnss/gnss_constants.hpp"
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"

namespace apollo {
namespace localization {
namespace local_gnss {

template <typename T>
class GnssPvt {
 public:
  explicit GnssPvt(const unsigned int max_gnss_prn);
  explicit GnssPvt(const GnssType& gnss_type);
  GnssPvt(const GnssType& gnss_type, const unsigned int max_gnss_prn);
  GnssPvt();
  virtual inline ~GnssPvt() {}

 public:
  inline bool SetEphMaxsize(const unsigned int max_size) {
    max_ephsize_gnss_ = max_size;
    return true;
  }

  inline bool SetEphMaxPrn(const unsigned int max_prn) {
    max_prn_ = max_prn;
    return true;
  }

  inline void SetGnssType(const GnssType& gnss_type) { gnss_type_ = gnss_type; }

  inline double GetEarthGm() { return gnss_ellipsoid_.GetEarthGm(); }

  inline double GetEarthSemiMajor() {
    return gnss_ellipsoid_.GetEarthSemiMajor();
  }

  inline double GetEarthRotation() {
    return gnss_ellipsoid_.GetEarthRotation();
  }

  inline double GetEarthJ20() { return gnss_ellipsoid_.GetEarthJ20(); }
  bool IsGEO(const unsigned int sat_prn) {
    if (gnss_type_ != apollo::drivers::gnss::BDS_SYS) {
      return false;
    }
    if (sat_prn > 5) {
      return false;
    }
    return true;
  }

  // GPS
  bool SaveEph(const T& current_eph);
  //
  bool GetPosVelClock(const unsigned int sat_prn,
                      const unsigned int gnss_week_num,
                      const double gps_time_signal_transimited,
                      const double time_signal_duration,
                      PointThreeDim* position, PointThreeDim* velocity,
                      double* clk_bias, double* clk_drift, double* eph_toe);
  virtual bool GetPositionVelocity(const double observe_time,
                                   const T& current_eph,
                                   PointThreeDim* position,
                                   PointThreeDim* velocity);

  virtual bool GetClock(const double observe_time, const T& current_eph,
                        double* clk_bias, double* clk_drift);
  inline virtual bool TransferPz90ToWgs84(PointThreeDim* position,
                                          PointThreeDim* velocity) {
    // override in glonass
    return true;
  }

  inline void CheckValidReferIOD(double* tk) {
    if (*tk > 302400) {
      *tk -= 604800;
    } else {
      if (*tk < -302400) {
        *tk += 604800;
      }
    }
  }

  inline double ComputeEhpEk(const double mk, const double eph_e,
                             const int max_iter_num) {
    double ek1 = mk;
    int cyclenum = 0;
    do {
      double ek2 = mk + eph_e * sin(ek1);
      double ek_err = fabs(ek2 - ek1);
      if ((ek_err <= 1.0e-12) || (cyclenum > max_iter_num)) {
        break;
      } else {
        ek1 = ek2;
      }
      cyclenum++;
    } while (1);
    return ek1;
  }

 private:
  void Initialize();

  bool IsGlonass() { return gnss_type_ == apollo::drivers::gnss::GLO_SYS; }

  bool SortCorrectEphemeris(const unsigned int sat_prn,
                            const double observe_time, T* time_related_eph);

  inline void AllocateEllipsoidParam(
      const GnssType& gnss_type = apollo::drivers::gnss::SYS_UNKNOWN) {
    GnssEllipsoidParam* p_temp = NULL;
    switch (gnss_type) {
      case apollo::drivers::gnss::GPS_SYS:
        p_temp = new GpsEllipsoidParam();
        break;
      case apollo::drivers::gnss::BDS_SYS:
        p_temp = new BdsEllipsoidParam();
        break;
      case apollo::drivers::gnss::GLO_SYS:
        p_temp = new GloEllipsoidParam();
        break;
      case apollo::drivers::gnss::GAL_SYS:
        p_temp = new GalEllipsoidParam();
        break;
      default:
        p_temp = new GnssEllipsoidParam();
        break;
    }
    if (p_temp != NULL) {
      gnss_ellipsoid_.SetEarthParams(
          p_temp->GetEarthSemiMajor(), p_temp->GetEarthEccentricity(),
          p_temp->GetEarthGm(), p_temp->GetEarthRotation(),
          p_temp->GetLightSpeed());
      delete p_temp;
    }
  }

 private:
  std::vector<std::vector<T>> _gnss_eph;
  std::vector<unsigned int> current_eph_index_;
  std::vector<double> latest_eph_toe_;

  GnssEllipsoidParam gnss_ellipsoid_;
  GnssType gnss_type_ = apollo::drivers::gnss::SYS_UNKNOWN;
  // set to 1(default) for real-time while 48 (recommended) for post-process
  unsigned int max_ephsize_gnss_ = 1;
  unsigned int max_prn_ = 99;
};

template <typename T>
GnssPvt<T>::GnssPvt(const unsigned int max_gnss_prn)
    : gnss_type_(apollo::drivers::gnss::GPS_SYS), max_prn_(max_gnss_prn) {
  Initialize();
}

template <typename T>
GnssPvt<T>::GnssPvt(const GnssType& gnss_type)
    : gnss_type_(gnss_type), max_prn_(1) {
  Initialize();
}

template <typename T>
GnssPvt<T>::GnssPvt(const GnssType& gnss_type, const unsigned int max_gnss_prn)
    : gnss_type_(gnss_type), max_prn_(max_gnss_prn) {
  Initialize();
}

template <typename T>
GnssPvt<T>::GnssPvt()
    : gnss_type_(apollo::drivers::gnss::GPS_SYS), max_prn_(32) {
  Initialize();
}

template <typename T>
void GnssPvt<T>::Initialize() {
  const unsigned int max_gps_prn = max_prn_ + 1;
  _gnss_eph.resize(max_gps_prn);
  current_eph_index_.resize(max_gps_prn);
  latest_eph_toe_.resize(max_gps_prn);
  for (unsigned int i = 0; i < max_gps_prn; ++i) {
    latest_eph_toe_[i] = -0.1;
  }
  AllocateEllipsoidParam(gnss_type_);
}

template <typename T>
bool GnssPvt<T>::SaveEph(const T& current_eph) {
  unsigned int sat_prn = current_eph.sat_prn();
  if (sat_prn <= 0 || sat_prn > max_prn_) {
    return false;
  }
  // new arrival toe check
  const double sec_per_week = apollo::localization::local_gnss::SECOND_PER_WEEK;
  double current_toe =
      current_eph.toe() + current_eph.week_num() * sec_per_week;
  if (current_toe <= latest_eph_toe_[sat_prn]) {
    // return false;
  }
  latest_eph_toe_[sat_prn] = current_toe;
  _gnss_eph[sat_prn].push_back(current_eph);
  while (_gnss_eph[sat_prn].size() > max_ephsize_gnss_) {
    _gnss_eph[sat_prn].erase(_gnss_eph[sat_prn].begin());
  }
  return true;
}

template <typename T>
bool GnssPvt<T>::GetPositionVelocity(const double observe_time,
                                     const T& current_eph,
                                     PointThreeDim* position,
                                     PointThreeDim* velocity) {
  return false;
}

template <typename T>
bool GnssPvt<T>::GetClock(const double observe_time, const T& current_eph,
                          double* clk_bias, double* clk_drift) {
  return false;
}

template <typename T>
bool GnssPvt<T>::GetPosVelClock(const unsigned int sat_prn,
                                const unsigned int gnss_week_num,
                                const double gps_time_signal_transimited,
                                const double time_signal_duration,
                                PointThreeDim* position,
                                PointThreeDim* velocity, double* clk_bias,
                                double* clk_drift, double* eph_toe) {
  double observe_time =
      gps_time_signal_transimited +
      apollo::localization::local_gnss::SECOND_PER_WEEK * gnss_week_num;
  // find time related GPS ephemeris
  T current_eph;
  double sort_time = observe_time;
  if (*eph_toe >= 0.0) {
    // find toe-specified eph
    sort_time = *eph_toe;
  }
  if (SortCorrectEphemeris(sat_prn, sort_time, &current_eph) == false) {
    return false;
  }
  if (current_eph.health() != 0) {
    return false;
  }

  *eph_toe = current_eph.week_num() *
             apollo::localization::local_gnss::SECOND_PER_WEEK +
             current_eph.toe();
  GetClock(observe_time, current_eph, clk_bias, clk_drift);
  GetPositionVelocity(observe_time, current_eph, position, velocity);

  double ang_velocity = gnss_ellipsoid_.GetEarthRotation();
  double alfa = ang_velocity * time_signal_duration;

  double cosa = cos(alfa);
  double sina = sin(alfa);
  double x = 0;
  double y = 0;
  x = position->x;
  y = position->y;
  position->x = x * cosa + y * sina;
  position->y = -x * sina + y * cosa;

  x = velocity->x;
  y = velocity->y;
  velocity->x = x * cosa + y * sina;
  velocity->y = -x * sina + y * cosa;

  if (IsGlonass()) {
    double relativity = 0.0;
    relativity = position->x * velocity->x + position->y * velocity->y +
                 position->z * velocity->z;
    relativity *= -2.0 / gnss_ellipsoid_.GetLightSpeed();
    *clk_bias += relativity / gnss_ellipsoid_.GetLightSpeed();
    TransferPz90ToWgs84(position, velocity);
  }

  return true;
}

template <typename T>
bool GnssPvt<T>::SortCorrectEphemeris(const unsigned int sat_prn,
                                      const double observe_time,
                                      T* time_related_eph) {
  if (_gnss_eph.size() < sat_prn || current_eph_index_.size() < sat_prn) {
    return false;
  }
  const std::vector<T>& prn_eph = _gnss_eph[sat_prn];
  if (prn_eph.size() <= 0) {
    return false;
  }
  *time_related_eph = prn_eph[current_eph_index_[sat_prn]];
  if (current_eph_index_[sat_prn] == prn_eph.size() - 1) {
    return true;
  }

  double toe0 = 0;
  double toe1 = 0;
  T temp0 = prn_eph[current_eph_index_[sat_prn]];
  toe0 = temp0.week_num() * apollo::localization::local_gnss::SECOND_PER_WEEK +
         temp0.toe();
  if (observe_time <= toe0) {
    return true;
  }
  for (unsigned int i = current_eph_index_[sat_prn] + 1;
       i <= prn_eph.size() - 1; ++i) {
    T temp1;
    temp1 = prn_eph[i];
    toe1 =
        temp1.week_num() * apollo::localization::local_gnss::SECOND_PER_WEEK +
        temp1.toe();
    if (observe_time - toe0 >= 0 && observe_time - toe1 < 0) {
      current_eph_index_[sat_prn] = i;
      *time_related_eph = prn_eph[current_eph_index_[sat_prn]];
      return true;
    }
    toe0 = toe1;
  }
  if (observe_time > toe1 && toe1 >= toe0) {
    current_eph_index_[sat_prn] = prn_eph.size() - 1;
    *time_related_eph = prn_eph[prn_eph.size() - 1];
    return true;
  }
  return true;
}

class KepplerPvt : public GnssPvt<KepplerOrbit> {
 public:
  inline explicit KepplerPvt(const unsigned int max_gnss_prn)
      : GnssPvt<KepplerOrbit>(apollo::drivers::gnss::GPS_SYS, max_gnss_prn) {}
  inline KepplerPvt(const GnssType& gnss_type, const unsigned int max_gnss_prn)
      : GnssPvt<KepplerOrbit>(gnss_type, max_gnss_prn) {}
  inline KepplerPvt()
      : GnssPvt<KepplerOrbit>(apollo::drivers::gnss::GPS_SYS, 32) {}
  virtual bool GetPositionVelocity(const double observe_time,
                                   const KepplerOrbit& current_eph,
                                   PointThreeDim* position,
                                   PointThreeDim* velocity);

  virtual bool GetClock(const double observe_time,
                        const KepplerOrbit& current_eph, double* clk_bias,
                        double* clk_drift);
};

class GPSPvt : public KepplerPvt {
 public:
  inline explicit GPSPvt(const unsigned int max_ephsize_gps)
      : KepplerPvt(apollo::drivers::gnss::GPS_SYS, 32) {
    SetEphMaxsize(max_ephsize_gps);
  }
  inline GPSPvt() : KepplerPvt(apollo::drivers::gnss::GPS_SYS, 32) {
    SetEphMaxsize(1);
  }
};

class BeidouPvt : public KepplerPvt {
 public:
  inline explicit BeidouPvt(const unsigned int max_ephsize_bds)
      : KepplerPvt(apollo::drivers::gnss::BDS_SYS, 37) {
    SetEphMaxsize(max_ephsize_bds);
  }
  inline BeidouPvt() : KepplerPvt(apollo::drivers::gnss::BDS_SYS, 37) {
    SetEphMaxsize(1);
  }
};

class GalileoPvt : public KepplerPvt {
 public:
  inline explicit GalileoPvt(const unsigned int max_ephsize_gal)
      : KepplerPvt(apollo::drivers::gnss::GAL_SYS, 32) {
    SetEphMaxsize(max_ephsize_gal);
  }
  inline GalileoPvt() : KepplerPvt(apollo::drivers::gnss::GAL_SYS, 32) {
    SetEphMaxsize(1);
  }
};

class GlonassPvt : public GnssPvt<GlonassOrbit> {
 public:
  inline explicit GlonassPvt(const unsigned int max_ephsize_glo)
      : GnssPvt<GlonassOrbit>(apollo::drivers::gnss::GLO_SYS, 32) {
    SetEphMaxsize(max_ephsize_glo);
  }
  inline GlonassPvt()
      : GnssPvt<GlonassOrbit>(apollo::drivers::gnss::GLO_SYS, 32) {
    SetEphMaxsize(1);
  }

  virtual bool GetClock(const double observe_time,
                        const GlonassOrbit& current_eph, double* clk_bias,
                        double* clk_drift);
  virtual bool GetPositionVelocity(const double observe_time,
                                   const GlonassOrbit& current_eph,
                                   PointThreeDim* position,
                                   PointThreeDim* velocity);
  bool compute_right_function(double pos[3], double vel[3], double acc[3],
                              double vel_acc_next[6]);

  bool position_pz90_to_wgs84(double pz90[3], double wgs84[3]);
  bool velocity_pz90_to_wgs84(double pz90[3], double wgs84[3]);

  bool TransferPz90ToWgs84(PointThreeDim* position, PointThreeDim* velocity);
};

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo

#endif
