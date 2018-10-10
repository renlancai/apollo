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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_RINEX_IO_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_RINEX_IO_H_

#include <fstream>
#include <vector>
#include "modules/drivers/gnss/proto/gnss_raw_observation.pb.h"
#include "modules/localization/msf/local_gnss/pvt_satellite.h"

namespace apollo {
namespace localization {
namespace local_gnss {

enum RINEX_VERSION {
  VER_UNKNOWN = 0,
  VERSION_2 = 2,
  VERSION_3 = 3,
  VERSION_SELF_DEFINE = 4
};
enum ValueType {
  VALUE_UNKNOWN = -1,
  VALUE_PHASE = 0,
  VALUE_RANGE = 1,
  VALUE_DOPPLER = 2,
  VALUE_SNR = 3
};

struct ObsTypeInfor {
  GnssType gnss_type;
  char indicator[4];
  GnssBandID band_id;
  ValueType v_type;
};

// should manually add new observation type here
const ObsTypeInfor g_obs_type_group_v2[] = {
    // gps L1
    {apollo::drivers::gnss::GPS_SYS, "C1", apollo::drivers::gnss::GPS_L1,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "P1", apollo::drivers::gnss::GPS_L1,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L1", apollo::drivers::gnss::GPS_L1,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D1", apollo::drivers::gnss::GPS_L1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S1", apollo::drivers::gnss::GPS_L1,
     VALUE_SNR},

    {apollo::drivers::gnss::GPS_SYS, "C2", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "P2", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L2", apollo::drivers::gnss::GPS_L2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D2", apollo::drivers::gnss::GPS_L2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S2", apollo::drivers::gnss::GPS_L2,
     VALUE_SNR},

    {apollo::drivers::gnss::GPS_SYS, "C3", apollo::drivers::gnss::GPS_L5,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "P3", apollo::drivers::gnss::GPS_L5,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L3", apollo::drivers::gnss::GPS_L5,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D3", apollo::drivers::gnss::GPS_L5,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S3", apollo::drivers::gnss::GPS_L5,
     VALUE_SNR},

    {apollo::drivers::gnss::GPS_SYS, "C5", apollo::drivers::gnss::GPS_L5,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "P5", apollo::drivers::gnss::GPS_L5,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L5", apollo::drivers::gnss::GPS_L5,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D5", apollo::drivers::gnss::GPS_L5,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S5", apollo::drivers::gnss::GPS_L5,
     VALUE_SNR},

    // bds
    {apollo::drivers::gnss::BDS_SYS, "C1", apollo::drivers::gnss::BDS_B1,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "P1", apollo::drivers::gnss::BDS_B1,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L1", apollo::drivers::gnss::BDS_B1,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D1", apollo::drivers::gnss::BDS_B1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S1", apollo::drivers::gnss::BDS_B1,
     VALUE_SNR},

    {apollo::drivers::gnss::BDS_SYS, "C2", apollo::drivers::gnss::BDS_B2,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "P2", apollo::drivers::gnss::BDS_B2,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L2", apollo::drivers::gnss::BDS_B2,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D2", apollo::drivers::gnss::BDS_B2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S2", apollo::drivers::gnss::BDS_B2,
     VALUE_SNR},

    {apollo::drivers::gnss::BDS_SYS, "C3", apollo::drivers::gnss::BDS_B3,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "P3", apollo::drivers::gnss::BDS_B3,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L3", apollo::drivers::gnss::BDS_B3,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D3", apollo::drivers::gnss::BDS_B3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S3", apollo::drivers::gnss::BDS_B3,
     VALUE_SNR},

    {apollo::drivers::gnss::BDS_SYS, "C5", apollo::drivers::gnss::BDS_B3,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "P5", apollo::drivers::gnss::BDS_B3,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L5", apollo::drivers::gnss::BDS_B3,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D5", apollo::drivers::gnss::BDS_B3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S5", apollo::drivers::gnss::BDS_B3,
     VALUE_SNR},

    // glonass
    {apollo::drivers::gnss::GLO_SYS, "C1", apollo::drivers::gnss::GLO_G1,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "P1", apollo::drivers::gnss::GLO_G1,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L1", apollo::drivers::gnss::GLO_G1,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D1", apollo::drivers::gnss::GLO_G1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S1", apollo::drivers::gnss::GLO_G1,
     VALUE_SNR},

    {apollo::drivers::gnss::GLO_SYS, "C2", apollo::drivers::gnss::GLO_G2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "P2", apollo::drivers::gnss::GLO_G2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L2", apollo::drivers::gnss::GLO_G2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D2", apollo::drivers::gnss::GLO_G2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S2", apollo::drivers::gnss::GLO_G2,
     VALUE_SNR},

    {apollo::drivers::gnss::GLO_SYS, "C3", apollo::drivers::gnss::GLO_G3,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "P3", apollo::drivers::gnss::GLO_G3,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L3", apollo::drivers::gnss::GLO_G3,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D3", apollo::drivers::gnss::GLO_G3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S3", apollo::drivers::gnss::GLO_G3,
     VALUE_SNR},

    {apollo::drivers::gnss::GLO_SYS, "C5", apollo::drivers::gnss::GLO_G3,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "P5", apollo::drivers::gnss::GLO_G3,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L5", apollo::drivers::gnss::GLO_G3,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D5", apollo::drivers::gnss::GLO_G3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S5", apollo::drivers::gnss::GLO_G3,
     VALUE_SNR}};
const ObsTypeInfor g_obs_type_group_v3[] = {
    // gps
    // L1 CA-code
    {apollo::drivers::gnss::GPS_SYS, "C1C", apollo::drivers::gnss::GPS_L1,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L1C", apollo::drivers::gnss::GPS_L1,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D1C", apollo::drivers::gnss::GPS_L1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S1C", apollo::drivers::gnss::GPS_L1,
     VALUE_SNR},
    // L2C(C)
    {apollo::drivers::gnss::GPS_SYS, "C2C", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L2C", apollo::drivers::gnss::GPS_L2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D2C", apollo::drivers::gnss::GPS_L2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S2C", apollo::drivers::gnss::GPS_L2,
     VALUE_SNR},
    // L2(D)
    {apollo::drivers::gnss::GPS_SYS, "C2D", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L2D", apollo::drivers::gnss::GPS_L2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D2D", apollo::drivers::gnss::GPS_L2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S2D", apollo::drivers::gnss::GPS_L2,
     VALUE_SNR},
    // L2C(M)
    {apollo::drivers::gnss::GPS_SYS, "C2S", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L2S", apollo::drivers::gnss::GPS_L2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D2S", apollo::drivers::gnss::GPS_L2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S2S", apollo::drivers::gnss::GPS_L2,
     VALUE_SNR},
    // L2 P-code
    {apollo::drivers::gnss::GPS_SYS, "C2P", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L2P", apollo::drivers::gnss::GPS_L2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D2P", apollo::drivers::gnss::GPS_L2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S2P", apollo::drivers::gnss::GPS_L2,
     VALUE_SNR},
    // L2 W-code
    {apollo::drivers::gnss::GPS_SYS, "C2W", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L2W", apollo::drivers::gnss::GPS_L2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D2W", apollo::drivers::gnss::GPS_L2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S2W", apollo::drivers::gnss::GPS_L2,
     VALUE_SNR},
    // L2 Y-code
    {apollo::drivers::gnss::GPS_SYS, "C2Y", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L2Y", apollo::drivers::gnss::GPS_L2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D2Y", apollo::drivers::gnss::GPS_L2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S2Y", apollo::drivers::gnss::GPS_L2,
     VALUE_SNR},
    // L2 X
    {apollo::drivers::gnss::GPS_SYS, "C2X", apollo::drivers::gnss::GPS_L2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L2X", apollo::drivers::gnss::GPS_L2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D2X", apollo::drivers::gnss::GPS_L2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S2X", apollo::drivers::gnss::GPS_L2,
     VALUE_SNR},
    // L5 I
    {apollo::drivers::gnss::GPS_SYS, "C5I", apollo::drivers::gnss::GPS_L5,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L5I", apollo::drivers::gnss::GPS_L5,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D5I", apollo::drivers::gnss::GPS_L5,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S5I", apollo::drivers::gnss::GPS_L5,
     VALUE_SNR},
    // L5 Q
    {apollo::drivers::gnss::GPS_SYS, "C5Q", apollo::drivers::gnss::GPS_L5,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L5Q", apollo::drivers::gnss::GPS_L5,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D5Q", apollo::drivers::gnss::GPS_L5,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S5Q", apollo::drivers::gnss::GPS_L5,
     VALUE_SNR},
    // L5 I+Q
    {apollo::drivers::gnss::GPS_SYS, "C5X", apollo::drivers::gnss::GPS_L5,
     VALUE_RANGE},
    {apollo::drivers::gnss::GPS_SYS, "L5X", apollo::drivers::gnss::GPS_L5,
     VALUE_PHASE},
    {apollo::drivers::gnss::GPS_SYS, "D5X", apollo::drivers::gnss::GPS_L5,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GPS_SYS, "S5X", apollo::drivers::gnss::GPS_L5,
     VALUE_SNR},
    // bds
    // B1
    {apollo::drivers::gnss::BDS_SYS, "C1I", apollo::drivers::gnss::BDS_B1,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L1I", apollo::drivers::gnss::BDS_B1,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D1I", apollo::drivers::gnss::BDS_B1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S1I", apollo::drivers::gnss::BDS_B1,
     VALUE_SNR},
    // B1 I
    {apollo::drivers::gnss::BDS_SYS, "C2I", apollo::drivers::gnss::BDS_B1,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L2I", apollo::drivers::gnss::BDS_B1,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D2I", apollo::drivers::gnss::BDS_B1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S2I", apollo::drivers::gnss::BDS_B1,
     VALUE_SNR},
    // B1 Q
    {apollo::drivers::gnss::BDS_SYS, "C2Q", apollo::drivers::gnss::BDS_B1,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L2Q", apollo::drivers::gnss::BDS_B1,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D2Q", apollo::drivers::gnss::BDS_B1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S2Q", apollo::drivers::gnss::BDS_B1,
     VALUE_SNR},
    // B1 I+Q=X
    {apollo::drivers::gnss::BDS_SYS, "C2X", apollo::drivers::gnss::BDS_B1,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L2X", apollo::drivers::gnss::BDS_B1,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D2X", apollo::drivers::gnss::BDS_B1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S2X", apollo::drivers::gnss::BDS_B1,
     VALUE_SNR},
    // B2 I
    {apollo::drivers::gnss::BDS_SYS, "C7I", apollo::drivers::gnss::BDS_B2,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L7I", apollo::drivers::gnss::BDS_B2,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D7I", apollo::drivers::gnss::BDS_B2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S7I", apollo::drivers::gnss::BDS_B2,
     VALUE_SNR},
    // B2 Q
    {apollo::drivers::gnss::BDS_SYS, "C7Q", apollo::drivers::gnss::BDS_B2,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L7Q", apollo::drivers::gnss::BDS_B2,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D7Q", apollo::drivers::gnss::BDS_B2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S7Q", apollo::drivers::gnss::BDS_B2,
     VALUE_SNR},
    // B2 I+Q=X
    {apollo::drivers::gnss::BDS_SYS, "C7X", apollo::drivers::gnss::BDS_B2,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L7X", apollo::drivers::gnss::BDS_B2,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D7X", apollo::drivers::gnss::BDS_B2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S7X", apollo::drivers::gnss::BDS_B2,
     VALUE_SNR},
    // B3 I
    {apollo::drivers::gnss::BDS_SYS, "C6I", apollo::drivers::gnss::BDS_B3,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L6I", apollo::drivers::gnss::BDS_B3,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D6I", apollo::drivers::gnss::BDS_B3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S6I", apollo::drivers::gnss::BDS_B3,
     VALUE_SNR},
    // B3 Q
    {apollo::drivers::gnss::BDS_SYS, "C6Q", apollo::drivers::gnss::BDS_B3,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L6Q", apollo::drivers::gnss::BDS_B3,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D6Q", apollo::drivers::gnss::BDS_B3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S6Q", apollo::drivers::gnss::BDS_B3,
     VALUE_SNR},
    // B3 X
    {apollo::drivers::gnss::BDS_SYS, "C6X", apollo::drivers::gnss::BDS_B3,
     VALUE_RANGE},
    {apollo::drivers::gnss::BDS_SYS, "L6X", apollo::drivers::gnss::BDS_B3,
     VALUE_PHASE},
    {apollo::drivers::gnss::BDS_SYS, "D6X", apollo::drivers::gnss::BDS_B3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::BDS_SYS, "S6X", apollo::drivers::gnss::BDS_B3,
     VALUE_SNR},
    // glonass
    // G1 CA-code
    {apollo::drivers::gnss::GLO_SYS, "C1C", apollo::drivers::gnss::GLO_G1,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L1C", apollo::drivers::gnss::GLO_G1,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D1C", apollo::drivers::gnss::GLO_G1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S1C", apollo::drivers::gnss::GLO_G1,
     VALUE_SNR},
    // G1 P-code
    {apollo::drivers::gnss::GLO_SYS, "C1P", apollo::drivers::gnss::GLO_G1,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L1P", apollo::drivers::gnss::GLO_G1,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D1P", apollo::drivers::gnss::GLO_G1,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S1P", apollo::drivers::gnss::GLO_G1,
     VALUE_SNR},
    // G2 C-code
    {apollo::drivers::gnss::GLO_SYS, "C2C", apollo::drivers::gnss::GLO_G2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L2C", apollo::drivers::gnss::GLO_G2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D2C", apollo::drivers::gnss::GLO_G2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S2C", apollo::drivers::gnss::GLO_G2,
     VALUE_SNR},
    // G2 P-code
    {apollo::drivers::gnss::GLO_SYS, "C2P", apollo::drivers::gnss::GLO_G2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L2P", apollo::drivers::gnss::GLO_G2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D2P", apollo::drivers::gnss::GLO_G2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S2P", apollo::drivers::gnss::GLO_G2,
     VALUE_SNR},
    // G2 W-code
    {apollo::drivers::gnss::GLO_SYS, "C2W", apollo::drivers::gnss::GLO_G2,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L2W", apollo::drivers::gnss::GLO_G2,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D2W", apollo::drivers::gnss::GLO_G2,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S2W", apollo::drivers::gnss::GLO_G2,
     VALUE_SNR},
    // G3 I
    {apollo::drivers::gnss::GLO_SYS, "C3I", apollo::drivers::gnss::GLO_G3,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L3I", apollo::drivers::gnss::GLO_G3,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D3I", apollo::drivers::gnss::GLO_G3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S3I", apollo::drivers::gnss::GLO_G3,
     VALUE_SNR},
    // G3 Q
    {apollo::drivers::gnss::GLO_SYS, "C3Q", apollo::drivers::gnss::GLO_G3,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L3Q", apollo::drivers::gnss::GLO_G3,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D3Q", apollo::drivers::gnss::GLO_G3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S3Q", apollo::drivers::gnss::GLO_G3,
     VALUE_SNR},
    // G3 I+Q
    {apollo::drivers::gnss::GLO_SYS, "C3X", apollo::drivers::gnss::GLO_G3,
     VALUE_RANGE},
    {apollo::drivers::gnss::GLO_SYS, "L3X", apollo::drivers::gnss::GLO_G3,
     VALUE_PHASE},
    {apollo::drivers::gnss::GLO_SYS, "D3X", apollo::drivers::gnss::GLO_G3,
     VALUE_DOPPLER},
    {apollo::drivers::gnss::GLO_SYS, "S3X", apollo::drivers::gnss::GLO_G3,
     VALUE_SNR},

    {apollo::drivers::gnss::GPS_SYS, "unk", apollo::drivers::gnss::GPS_L1,
     VALUE_UNKNOWN}};

class RinexNav {
 public:
  RinexNav();
  RinexNav(const char* nav_file, const RINEX_VERSION& version);
  ~RinexNav();
  // bool open_file(const char* nav_file);
  inline bool IsFileOpen() { return fp_nav_.is_open(); }
  inline void SetGnssTypeForRinex2(const GnssType& gnss_type) {
    gnss_type_ = gnss_type;
  }
  inline bool IsFileEof() { return fp_nav_.eof(); }
  inline bool FindSubString(const char* dest, const char* sub_string) {
    return (strstr(dest, sub_string) != NULL);
  }
  bool ReadEphEpoch(
      apollo::localization::local_gnss::GnssEphemeris* gnss_orbit);

  inline int GetLeapSecond() { return leap_second_s_; }

  static bool WriteEph(bool header_write, const char* eph_file,
                       const GnssEphemeris& raw_eph);

  static bool WriteEph(bool header_write, FILE* fp_eph,
                       const GnssEphemeris& raw_eph);
  inline void set_header_loaded(bool flag) { b_header_loaded_ = flag; }

 private:
  void Initialize();
  bool LoadHeader();
  bool ReadRinexVer3(GnssEphemeris* gnss_orbit);

  // bool ReadRinexVer2(GnssEphemeris* gnss_orbit);

  bool ReadEphContext(GnssEphemeris* gnss_orbit);

  GnssType GetSatSysFromCharacter(char sys_ch);

 private:
  std::fstream fp_nav_;
  RINEX_VERSION rinex_ver_;
  bool b_header_loaded_;
  // when under version 2, the GnssType should be set manually
  GnssType gnss_type_;
  // version 2.x differs from vebit_bias_rsion 3.x only in one-bit bias
  unsigned int bit_bias_;
  int leap_second_s_;
};

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo

#endif
