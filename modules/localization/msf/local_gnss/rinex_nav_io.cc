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

#include <string>
#include "modules/localization/msf/local_gnss/gnss_utility.hpp"
#include "modules/localization/msf/local_gnss/rinex_io.h"

namespace apollo {
namespace localization {
namespace local_gnss {

#define ONLINE_RINEX_NAV

RinexNav::RinexNav() {
  apollo::localization::local_gnss::RinexNav::Initialize();
  rinex_ver_ = apollo::localization::local_gnss::VERSION_2;
}

RinexNav::RinexNav(const char* nav_file, const RINEX_VERSION& version) {
  Initialize();
  if (nav_file != NULL) {
    fp_nav_.open(nav_file, std::ios::in);
  }
  rinex_ver_ = version;
}

RinexNav::~RinexNav() {
  if (fp_nav_.is_open() == true) {
    fp_nav_.close();
  }
}

void RinexNav::Initialize() {
  b_header_loaded_ = false;
  gnss_type_ = apollo::drivers::gnss::GPS_SYS;
  leap_second_s_ = 999999;
  bit_bias_ = 0;
}

bool RinexNav::LoadHeader() {
  std::string str = "";
  const int buf_size = 256;
  char line[buf_size] = {'\0'};
  if (!fp_nav_) {
    return false;
  }
  // load header
  if (!b_header_loaded_) {
    while (!fp_nav_.eof()) {
      fp_nav_.getline(line, buf_size);
      str = line;
      if (FindSubString(line, "END OF HEADER")) {
        b_header_loaded_ = true;
        break;
      } else if (FindSubString(line, "LEAP SECONDS")) {
        leap_second_s_ = atoi(str.substr(4, 3).c_str());
      }
    }
  }
  b_header_loaded_ = true;
  return true;
}

bool RinexNav::ReadEphEpoch(
    apollo::localization::local_gnss::GnssEphemeris* gnss_orbit) {
  if (fp_nav_.is_open() == false) {
    return false;
  }
  bool b_flag = false;
  if (!b_header_loaded_) {
    LoadHeader();
  }
  if (rinex_ver_ == apollo::localization::local_gnss::VERSION_3) {
    b_flag = ReadRinexVer3(gnss_orbit);
  } else if (rinex_ver_ == apollo::localization::local_gnss::VERSION_2) {
    // b_flag = read_v2(gnss_orbit);
  }
  switch (gnss_orbit->gnss_type()) {
    case apollo::drivers::gnss::GPS_SYS:
      gnss_orbit->mutable_keppler_orbit()->set_gnss_time_type(
          apollo::drivers::gnss::GPS_TIME);
      break;
    case apollo::drivers::gnss::BDS_SYS:
      gnss_orbit->mutable_keppler_orbit()->set_gnss_time_type(
          apollo::drivers::gnss::BDS_TIME);
      break;
    case apollo::drivers::gnss::GLO_SYS:
      gnss_orbit->mutable_glonass_orbit()->set_gnss_time_type(
          apollo::drivers::gnss::GLO_TIME);
      break;
    default:
      break;
  }
  return b_flag;
}

bool RinexNav::WriteEph(bool header_write, const char* eph_file,
                        const GnssEphemeris& raw_eph) {
  if (eph_file == NULL) {
    return false;
  }
  FILE* fp = fopen(eph_file, "a+");
  if (fp == NULL) {
    return false;
  }
  bool flag = WriteEph(header_write, fp, raw_eph);
  fflush(fp);
  fclose(fp);
  return flag;
}

bool RinexNav::WriteEph(bool header_write, FILE* fp_eph,
                        const GnssEphemeris& raw_eph) {
  if (fp_eph == NULL) {
    return false;
  }
  if (header_write) {
    fprintf(fp_eph, "END OF HEADER\n");
  }
  GlonassOrbit temp_glo;
  KepplerOrbit temp_kep;
  char sys_char = 'N';
  switch (raw_eph.gnss_type()) {
    case apollo::drivers::gnss::GPS_SYS:
      sys_char = 'G';
      break;
    case apollo::drivers::gnss::BDS_SYS:
      sys_char = 'C';
      break;
    case apollo::drivers::gnss::GLO_SYS:
      sys_char = 'R';
      break;
    default:
      return false;
  }
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int minute = 0;
  double second_s = 0.0;
  if (raw_eph.gnss_type() == apollo::drivers::gnss::GPS_SYS ||
      raw_eph.gnss_type() == apollo::drivers::gnss::BDS_SYS) {
    temp_kep = raw_eph.keppler_orbit();
    int week_num = temp_kep.week_num();
    if (temp_kep.gnss_type() == apollo::drivers::gnss::BDS_SYS) {
      /* BDS week_num has a const offset (1356 = 1953 - 597) from GPS's*/
      week_num += 1356;
    }
    gnss_utility::GpsTime2DayTime(week_num, temp_kep.toc(), &year, &month, &day,
                                  &hour, &minute, &second_s);
    // 1st line
    fprintf(fp_eph, "%c%02d%5d %02d %02d %02d %02d %02d%19.12e%19.12e%19.12e\n",
            sys_char, temp_kep.sat_prn(), year, month, day, hour, minute,
            static_cast<int>(second_s), temp_kep.af0(), temp_kep.af1(),
            temp_kep.af2());
    // 2nd
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n", temp_kep.iode(),
            temp_kep.crs(), temp_kep.deltan(), temp_kep.m0());
    // 3rd
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n", temp_kep.cuc(),
            temp_kep.e(), temp_kep.cus(), temp_kep.roota());
    // 4th
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n", temp_kep.toe(),
            temp_kep.cic(), temp_kep.omega0(), temp_kep.cis());
    // 5th
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n", temp_kep.i0(),
            temp_kep.crc(), temp_kep.omega(), temp_kep.omegadot());
    // 6th
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n", temp_kep.idot(),
            static_cast<double>(temp_kep.codesonl2channel()),
            static_cast<double>(temp_kep.week_num()),
            static_cast<double>(temp_kep.l2pdataflag()));
    // 7th
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n",
            static_cast<double>(temp_kep.accuracy()),
            static_cast<double>(temp_kep.health()), temp_kep.tgd(),
            static_cast<double>(temp_kep.iodc()));
    // 8th
    double reserved = 0.0;
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n", reserved, reserved,
            reserved, reserved);
  } else if (raw_eph.gnss_type() == apollo::drivers::gnss::GLO_SYS) {
    temp_glo = raw_eph.glonass_orbit();
    gnss_utility::GpsTime2DayTime(temp_glo.week_num(), temp_glo.toe(), &year,
                                  &month, &day, &hour, &minute, &second_s);
    // 1st line
    fprintf(fp_eph, "%c%02d%5d %02d %02d %02d %02d %02d%19.12e%19.12e%19.12e\n",
            sys_char, temp_glo.slot_prn(), year, month, day, hour, minute,
            static_cast<int>(second_s), temp_glo.clock_offset(),
            temp_glo.clock_drift(), temp_glo.tk());
    // 2nd
    const double km = 1000.0;
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n",
            temp_glo.position_x() / km, temp_glo.velocity_x() / km,
            temp_glo.accelerate_x() / km,
            static_cast<double>(temp_glo.health()));
    // 3rd
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n",
            temp_glo.position_y() / km, temp_glo.velocity_y() / km,
            temp_glo.accelerate_y() / km,
            static_cast<double>(temp_glo.frequency_no()));
    // 4th
    fprintf(fp_eph, "    ");
    fprintf(fp_eph, "%19.12e%19.12e%19.12e%19.12e\n",
            temp_glo.position_z() / km, temp_glo.velocity_z() / km,
            temp_glo.accelerate_z() / km, temp_glo.infor_age());
  }
  return true;
}

/*
bool RinexNav::open_file(const char* nav_file) {
  if (nav_file != NULL) {
    fp_nav_.open(nav_file, std::ios::in);
    return  fp_nav_.is_open();
  } else {
    return false;
  }
}
*/

bool RinexNav::ReadRinexVer3(GnssEphemeris* gnss_orbit) {
  bit_bias_ = 1;
  // gnss_orbit->gnss_type = gnss_type_;
  // gnss_orbit->gnss_type would be modified in the function
  return ReadEphContext(gnss_orbit);
}

/*
bool RinexNav::read_v2(apollo::localization::local_gnss::GnssEphemeris*
gnss_orbit) { bit_bias_ = 0; gnss_orbit->setgnss_type_(gnss_type_); return
ReadEphContext(gnss_orbit);
}
*/

bool RinexNav::ReadEphContext(GnssEphemeris* gnss_orbit) {
  std::string str = "";
  const int buf_size = 256;
  char line[buf_size] = {'\0'};
  if (!fp_nav_) {
    return false;
  }
  // load header
  if (!b_header_loaded_) {
    LoadHeader();
  }
  if (fp_nav_.eof()) {
    return false;
  }
  const int min_str_len = 80;
  // 1st line
  fp_nav_.getline(line, buf_size);
  str = line;
  if (str.length() != min_str_len) {
    return false;
  }
  // file ends with an empty line calling for careful process
  if (line[0] == '\0' && fp_nav_.eof()) {
    return false;
  }
  int first_linebit_bias_ = 0;
  int year = 0;
  if (bit_bias_ == 1) {
    // for V3, the first char should be a-GNSS indicator
    if (!(line[0] == 'G' || line[0] == 'R' || line[0] == 'C' ||
          line[0] == 'E' || line[0] == 'S' || line[0] == 'J')) {
      return false;
    }

    gnss_orbit->set_gnss_type(GetSatSysFromCharacter(line[0]));
    first_linebit_bias_ = 3;
    year = atoi((str.substr(4, 4)).c_str());
  }

  int month = atoi((str.substr(6 + first_linebit_bias_, 2)).c_str());
  int day = atoi((str.substr(9 + first_linebit_bias_, 2)).c_str());
  int hour = atoi((str.substr(12 + first_linebit_bias_, 2)).c_str());
  int minute = atoi((str.substr(15 + first_linebit_bias_, 2)).c_str());
  double second = atof((str.substr(18 + first_linebit_bias_, 10)).c_str());
  int week_num = 0;
  double week_second_s = 0.0;
  week_num = apollo::localization::local_gnss::gnss_utility::DayTime2GpsTime(
      year, month, day, hour, minute, second, &week_second_s);
  unsigned int bit = bit_bias_;
  if (line[0] == 'R' || line[1] == 'S') {
    gnss_orbit->mutable_glonass_orbit()->set_toe(week_second_s);
    // 1st line continue
    gnss_orbit->mutable_glonass_orbit()->set_slot_prn(
        atoi(str.substr(0 + bit, 2).c_str()));
    gnss_orbit->mutable_glonass_orbit()->set_clock_offset(
        atof(str.substr(22 + bit, 19).c_str()));
    gnss_orbit->mutable_glonass_orbit()->set_clock_drift(
        atof(str.substr(41 + bit, 19).c_str()));
    gnss_orbit->mutable_glonass_orbit()->set_tk(
        atof((str.substr(60 + bit, 19)).c_str()));
    gnss_orbit->mutable_glonass_orbit()->set_year(year);
    gnss_orbit->mutable_glonass_orbit()->set_month(month);
    gnss_orbit->mutable_glonass_orbit()->set_day(day);
    gnss_orbit->mutable_glonass_orbit()->set_hour(hour);
    gnss_orbit->mutable_glonass_orbit()->set_minute(minute);
    gnss_orbit->mutable_glonass_orbit()->set_second_s(second);
    gnss_orbit->mutable_glonass_orbit()->set_week_num(week_num);
    gnss_orbit->mutable_glonass_orbit()->set_week_second_s(week_second_s);
    // 2nd line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_glonass_orbit()->set_position_x(
        atof(str.substr(3 + bit, 19).c_str()) * 1000);
    gnss_orbit->mutable_glonass_orbit()->set_velocity_x(
        atof(str.substr(22 + bit, 19).c_str()) * 1000);
    gnss_orbit->mutable_glonass_orbit()->set_accelerate_x(
        atof(str.substr(41 + bit, 19).c_str()) * 1000);
    gnss_orbit->mutable_glonass_orbit()->set_health(
        atoi(str.substr(60 + bit, 19).c_str()));
    // 3rd line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_glonass_orbit()->set_position_y(
        atof(str.substr(3 + bit, 19).c_str()) * 1000);
    gnss_orbit->mutable_glonass_orbit()->set_velocity_y(
        atof(str.substr(22 + bit, 19).c_str()) * 1000);
    gnss_orbit->mutable_glonass_orbit()->set_accelerate_y(
        atof(str.substr(41 + bit, 19).c_str()) * 1000);
    double fre_no = atof(str.substr(60 + bit, 19).c_str());
    gnss_orbit->mutable_glonass_orbit()->set_frequency_no(
        static_cast<int>(fre_no));
    // 4th line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_glonass_orbit()->set_position_z(
        atof(str.substr(3 + bit, 19).c_str()) * 1000);
    gnss_orbit->mutable_glonass_orbit()->set_velocity_z(
        atof(str.substr(22 + bit, 19).c_str()) * 1000);
    gnss_orbit->mutable_glonass_orbit()->set_accelerate_z(
        atof(str.substr(41 + bit, 19).c_str()) * 1000);
    gnss_orbit->mutable_glonass_orbit()->set_infor_age(
        atoi(str.substr(60 + bit, 19).c_str()));
  } else {
    // gps or bds
    // 1st line continue
    gnss_orbit->mutable_keppler_orbit()->set_sat_prn(
        atoi(str.substr(0 + bit, 2).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_af0(
        atof(str.substr(22 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_af1(
        atof(str.substr(41 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_af2(
        atof(str.substr(60 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_year(year);
    gnss_orbit->mutable_keppler_orbit()->set_month(month);
    gnss_orbit->mutable_keppler_orbit()->set_day(day);
    gnss_orbit->mutable_keppler_orbit()->set_hour(hour);
    gnss_orbit->mutable_keppler_orbit()->set_minute(minute);
    gnss_orbit->mutable_keppler_orbit()->set_second_s(second);
    gnss_orbit->mutable_keppler_orbit()->set_week_num(week_num);
    // gnss_orbit->mutable_keppler_orbit()->set_week_second_s(week_second_s);
    // toc
    gnss_orbit->mutable_keppler_orbit()->set_toc(week_second_s);
    // 2nd line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_keppler_orbit()->set_iode(
        atof(str.substr(3 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_crs(
        atof(str.substr(22 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_deltan(
        atof(str.substr(41 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_m0(
        atof(str.substr(60 + bit, 19).c_str()));
    // 3rd line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_keppler_orbit()->set_cuc(
        atof(str.substr(3 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_e(
        atof(str.substr(22 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_cus(
        atof(str.substr(41 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_roota(
        atof(str.substr(60 + bit, 19).c_str()));
    // 4th line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_keppler_orbit()->set_toe(
        atof(str.substr(3 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_cic(
        atof(str.substr(22 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_omega0(
        atof(str.substr(41 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_cis(
        atof(str.substr(60 + bit, 19).c_str()));
    // 5th line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_keppler_orbit()->set_i0(
        atof(str.substr(3 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_crc(
        atof(str.substr(22 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_omega(
        atof(str.substr(41 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_omegadot(
        atof(str.substr(60 + bit, 19).c_str()));
    // 6th line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_keppler_orbit()->set_idot(
        atof(str.substr(3 + bit, 19).c_str()));
    // gnss_orbit->mutable_keppler_orbit()->set_codesonL2channel(
    //    atof(str.substr(22 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_week_num(
        (unsigned int)atof(str.substr(41 + bit, 19).c_str()));
    // gnss_orbit->mutable_keppler_orbit()->set_L2Pdataflag(atof(str.substr(60 +
    // bit, 19).c_str())); 7th line
    fp_nav_.getline(line, buf_size);
    str = line;
    if (str.length() != min_str_len) {
      return false;
    }
    gnss_orbit->mutable_keppler_orbit()->set_accuracy(
        atoi(str.substr(3 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_health(
        atoi(str.substr(22 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_tgd(
        atof(str.substr(41 + bit, 19).c_str()));
    gnss_orbit->mutable_keppler_orbit()->set_iodc(
        atof(str.substr(60 + bit, 19).c_str()));
    // 8th line
    fp_nav_.getline(line, buf_size);
    str = line;
    // gnss_orbit->mutable_keppler_orbit()->set_ttm(atof(str.substr(3 + bit,
    // 19).c_str()));
  }
  return true;
}

GnssType RinexNav::GetSatSysFromCharacter(char sys_ch) {
  if (sys_ch == 'R') {
    return apollo::drivers::gnss::GLO_SYS;
  } else if (sys_ch == 'G') {
    return apollo::drivers::gnss::GPS_SYS;
  } else if (sys_ch == 'B' || sys_ch == 'C') {
    return apollo::drivers::gnss::BDS_SYS;
  }
  return apollo::drivers::gnss::SYS_UNKNOWN;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
