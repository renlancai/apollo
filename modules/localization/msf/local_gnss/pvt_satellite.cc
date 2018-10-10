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

#include "modules/localization/msf/local_gnss/pvt_satellite.h"
#include <math.h>
namespace apollo {
namespace localization {
namespace local_gnss {
// ---------------- GPS --------------------------------------
// L1 carrier frequency in Hz
const double L1_FREQ_GPS = 1575.42e6;
// L2 carrier frequency in Hz
const double L2_FREQ_GPS = 1227.60e6;
// L5 carrier frequency in Hz.
const double L5_FREQ_GPS = 1176.45e6;
// L1 carrier wavelength in meters
const double L1_WAVELENGTH_GPS = 0.190293672798;
// L2 carrier wavelength in meters
const double L2_WAVELENGTH_GPS = 0.244210213425;
// L5 carrier wavelength in meters.
const double L5_WAVELENGTH_GPS = 0.254828049;

// ---------------- GLONASS ----------------------------------
// f1(n) = 1602.0e6 + n * 562.5e3 Hz = 9 * (178 + n*0.0625) MHz
// f2(n) = 1246.0e6 + n * 437.5e3 Hz = 7 * (178 + n*0.0625) MHz
// where n is the time- and satellite-dependent 'frequency channel' as -7 <= n
// <= 7

// L1 carrier base frequency in Hz.
const double L1_FREQ_GLO = 1602.0e6;
// L1 carrier frequency step size in Hz.
const double L1_FREQ_STEP_GLO = 562.5e3;
// L1 carrier wavelength in meters.
const double L1_WAVELENGTH_GLO = 0.187136365793;
// L2 carrier base frequency in Hz.
const double L2_FREQ_GLO = 1246.0e6;
// L2 carrier frequency step size in Hz.
const double L2_FREQ_STEP_GLO = 437.5e3;
// L2 carrier wavelength in meters.
const double L2_WAVELENGTH_GLO = 0.240603898876;

// ---------------- Galileo ----------------------------------
// L1 (E1) carrier frequency in Hz
const double L1_FREQ_GAL = L1_FREQ_GPS;
// L5 (E5a) carrier frequency in Hz.
const double L5_FREQ_GAL = L5_FREQ_GPS;
// L6 (E6) carrier frequency in Hz.
const double L6_FREQ_GAL = 1278.75e6;
// L7 (E5b) carrier frequency in Hz.
const double L7_FREQ_GAL = 1207.140e6;
// L8 (E5a+E5b) carrier frequency in Hz.
const double L8_FREQ_GAL = 1191.795e6;

// L1 carrier wavelength in meters
const double L1_WAVELENGTH_GAL = L1_WAVELENGTH_GPS;
// L5 carrier wavelength in meters.
const double L5_WAVELENGTH_GAL = L5_WAVELENGTH_GPS;
// L6 carrier wavelength in meters.
const double L6_WAVELENGTH_GAL = 0.234441805;
// L7 carrier wavelength in meters.
const double L7_WAVELENGTH_GAL = 0.24834937;
// L8 carrier wavelength in meters.
const double L8_WAVELENGTH_GAL = 0.251547001;

// ---------------- BeiDou ----------------------------------
// L1 (B1) carrier frequency in Hz.
const double L1_FREQ_BDS = 1561.098e6;
// L2 (B2) carrier frequency in Hz.
const double L2_FREQ_BDS = 1207.140e6;
// L3 (B3) carrier frequency in Hz.
const double L3_FREQ_BDS = 1268.52e6;

// L1 carrier wavelength in meters.
const double L1_WAVELENGTH_BDS = 0.192039486310276;
// L2 carrier wavelength in meters.
const double L2_WAVELENGTH_BDS = 0.24834937;
// L3 carrier wavelength in meters.
const double L3_WAVELENGTH_BDS = 0.236332464604421;

// ---------------- QZSS ----------------------------------
// QZS-L1 carrier frequency in Hz.
const double L1_FREQ_QZS = L1_FREQ_GPS;
// QZS-L2 carrier frequency in Hz.
const double L2_FREQ_QZS = L2_FREQ_GPS;
// QZS-L5 carrier frequency in Hz.
const double L5_FREQ_QZS = L5_FREQ_GPS;
// QZS-LEX(6) carrier frequency in Hz.
const double L6_FREQ_QZS = L6_FREQ_GAL;

// QZS-L1 carrier wavelength in meters.
const double L1_WAVELENGTH_QZS = L1_WAVELENGTH_GPS;
// QZS-L2 carrier wavelength in meters.
const double L2_WAVELENGTH_QZS = L2_WAVELENGTH_GPS;
// QZS-L5 carrier wavelength in meters.
const double L5_WAVELENGTH_QZS = L5_WAVELENGTH_GPS;
// QZS-LEX(6) carrier wavelength in meters.
const double L6_WAVELENGTH_QZS = L6_WAVELENGTH_GAL;

// ---------------- Geostationary (SBAS) ---------------------
// GEO-L1 carrier frequency in Hz
const double L1_FREQ_GEO = L1_FREQ_GPS;
// GEO-L5 carrier frequency in Hz.
const double L5_FREQ_GEO = L5_FREQ_GPS;

// GEO-L1 carrier wavelength in meters
const double L1_WAVELENGTH_GEO = L1_WAVELENGTH_GPS;
// GEO-L5 carrier wavelength in meters.
const double L5_WAVELENGTH_GEO = L5_WAVELENGTH_GPS;

int apollo_adu_local_gnss_glo_slot_frq[24] = {1,  -4, 5, 6,  1,  -4, 5, 6,
                                             -2, -7, 0, -1, -2, -7, 0, -1,
                                             4,  -3, 3, 2,  4,  -3, 3, 2};

SatelliteInterface::SatelliteInterface(const unsigned int max_size_eph_gps,
                                       const unsigned int max_size_eph_bds,
                                       const unsigned int max_size_eph_glo)
    : gps_eph_(max_size_eph_gps),
      bds_eph_(max_size_eph_bds),
      glo_eph_(max_size_eph_glo) {
  current_leap_second_s_ = 0;
  map_gnss_eph_.clear();
}

SatelliteInterface::SatelliteInterface() {
  current_leap_second_s_ = 0;
  map_gnss_eph_.clear();
}

SatelliteInterface::~SatelliteInterface() { map_gnss_eph_.clear(); }

bool SatelliteInterface::SetEphMaxsize(const int max_size,
                                          GnssType sat_sys) {
  if (sat_sys == apollo::drivers::gnss::GPS_SYS) {
    return gps_eph_.SetEphMaxsize(max_size);
  } else if (sat_sys == apollo::drivers::gnss::BDS_SYS) {
    return bds_eph_.SetEphMaxsize(max_size);
  } else if (sat_sys == apollo::drivers::gnss::GLO_SYS) {
    return glo_eph_.SetEphMaxsize(max_size);
  } else {
    return false;
  }
}

bool SatelliteInterface::CheckDuplicatedEph(const GnssEphemeris &raw_eph) {
  GnssType t_type = raw_eph.gnss_type();
  if (t_type != apollo::drivers::gnss::GPS_SYS &&
      t_type != apollo::drivers::gnss::BDS_SYS &&
      t_type != apollo::drivers::gnss::GLO_SYS) {
    return true;
  }
  unsigned int sat_prn = 0;
  unsigned int week_num = 0;
  double toe = -0.1;
  if (t_type == apollo::drivers::gnss::GLO_SYS) {
    sat_prn = raw_eph.glonass_orbit().slot_prn();
    week_num = raw_eph.glonass_orbit().week_num();
    toe = raw_eph.glonass_orbit().toe();
  } else {
    sat_prn = raw_eph.keppler_orbit().sat_prn();
    week_num = raw_eph.keppler_orbit().week_num();
    toe = raw_eph.keppler_orbit().toe();
  }
  const EphKey temp(t_type, sat_prn, week_num, toe);
  if (map_gnss_eph_.find(temp) != map_gnss_eph_.end()) {
    return true;
  }
  map_gnss_eph_.insert(
      std::map<EphKey, GnssEphemeris>::value_type(temp, raw_eph));
  return false;
}

bool SatelliteInterface::SaveEphemeris(const GnssEphemeris &gnss_orbit) {
  unsigned int slot_prn = 0;
  int fre_no = 0;
  if (CheckDuplicatedEph(gnss_orbit)) {
    // non-gnss or already saved eph
    return false;
  }
  switch (gnss_orbit.gnss_type()) {
    case apollo::drivers::gnss::GPS_SYS:
      gps_eph_.SaveEph(gnss_orbit.keppler_orbit());
      break;
    case apollo::drivers::gnss::BDS_SYS:
      bds_eph_.SaveEph(gnss_orbit.keppler_orbit());
      break;
    case apollo::drivers::gnss::GLO_SYS: {
      slot_prn = gnss_orbit.glonass_orbit().slot_prn();
      fre_no = gnss_orbit.glonass_orbit().frequency_no();
      GlonassOrbit temp_glo = gnss_orbit.glonass_orbit();
      temp_glo.set_sat_prn(slot_prn);
      glo_eph_.SaveEph(temp_glo);
      break;
    }
    default:
      break;
  }
  if (slot_prn > 0) {
    // update default frequency number
    apollo_adu_local_gnss_glo_slot_frq[slot_prn - 1] = fre_no;
  }
  return true;
}

bool SatelliteInterface::GetPosVelClock(
    const GnssType sat_sys, const int sat_prn, const unsigned int gnss_week_num,
    const double time_signal_transimited, const double time_signal_duration,
    PointThreeDim* position, PointThreeDim* velocity, double* clk_bias,
    double* clk_drift, double* eph_toe) {
  const unsigned int week_gap_bds = 1356;
  const double second_gap_bds = 14;
  // convert gnss time to utc time
  const double leap_second_s = current_leap_second_s_;
  // leap_second_s = 18.0;
  const unsigned int bd_week_num = gnss_week_num - week_gap_bds;
  const double bds_obs_time = time_signal_transimited - second_gap_bds;
  bool flag = false;
  switch (sat_sys) {
    case apollo::drivers::gnss::GPS_SYS:
      flag = gps_eph_.GetPosVelClock(
          sat_prn, gnss_week_num, time_signal_transimited, time_signal_duration,
          position, velocity, clk_bias, clk_drift, eph_toe);
      // flag = false;
      break;
    case apollo::drivers::gnss::BDS_SYS:
      flag = bds_eph_.GetPosVelClock(sat_prn, bd_week_num, bds_obs_time,
                                      time_signal_duration, position, velocity,
                                      clk_bias, clk_drift, eph_toe);
      // debug
      // flag = false;
      break;
    case apollo::drivers::gnss::GLO_SYS:
      flag = glo_eph_.GetPosVelClock(sat_prn, gnss_week_num,
                                      time_signal_transimited - leap_second_s,
                                      time_signal_duration, position, velocity,
                                      clk_bias, clk_drift, eph_toe);
      // debug
      // flag = false;
      break;
    default:
      break;
  }
  return flag;
}
double SatelliteInterface::GetBandLength(
    const int prn, const apollo::drivers::gnss::GnssBandID &fre_id,
    int* glo_fre_num) {
  double wl = -0.1;
  *glo_fre_num = 0;
  switch (fre_id) {
    case apollo::drivers::gnss::GPS_L1:
      wl = L1_WAVELENGTH_GPS;
      break;
    case apollo::drivers::gnss::GPS_L2:
      wl = L2_WAVELENGTH_GPS;
      break;
    case apollo::drivers::gnss::GPS_L5:
      wl = L5_WAVELENGTH_GPS;
      break;
    case apollo::drivers::gnss::GLO_G1:
      wl = (C_MPS / (L1_FREQ_GLO + apollo_adu_local_gnss_glo_slot_frq[prn - 1] *
                                       L1_FREQ_STEP_GLO));
      *glo_fre_num = apollo_adu_local_gnss_glo_slot_frq[prn - 1];
      break;
    case apollo::drivers::gnss::GLO_G2:
      wl = (C_MPS / (L2_FREQ_GLO + apollo_adu_local_gnss_glo_slot_frq[prn - 1] *
                                       L2_FREQ_STEP_GLO));
      *glo_fre_num = apollo_adu_local_gnss_glo_slot_frq[prn - 1];
      break;
    case apollo::drivers::gnss::BDS_B1:
      wl = L1_WAVELENGTH_BDS;
      break;
    case apollo::drivers::gnss::BDS_B2:
      wl = L2_WAVELENGTH_BDS;
      break;
    case apollo::drivers::gnss::BDS_B3:
      wl = L3_WAVELENGTH_BDS;
      break;
    default:
      break;
  }
  return wl;
}

GnssType SatelliteInterface::DetermineGnssType(
    const apollo::drivers::gnss::GnssBandID &band_id) {
  GnssType temp = apollo::drivers::gnss::SYS_UNKNOWN;
  if (band_id == apollo::drivers::gnss::GPS_L1 ||
      band_id == apollo::drivers::gnss::GPS_L2 ||
      band_id == apollo::drivers::gnss::GPS_L5) {
    temp = apollo::drivers::gnss::GPS_SYS;
  } else if (band_id == apollo::drivers::gnss::BDS_B1 ||
             band_id == apollo::drivers::gnss::BDS_B2 ||
             band_id == apollo::drivers::gnss::BDS_B3) {
    temp = apollo::drivers::gnss::BDS_SYS;
  } else if (band_id == apollo::drivers::gnss::GLO_G1 ||
             band_id == apollo::drivers::gnss::GLO_G2) {
    temp = apollo::drivers::gnss::GLO_SYS;
  } else {
    // TO DO: add interface for GALILEO/QZSS/IRNSS, etc.
  }
  return temp;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
