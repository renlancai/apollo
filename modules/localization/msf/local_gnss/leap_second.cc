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

#include "modules/localization/msf/local_gnss/leap_second.h"

#include "modules/localization/msf/local_gnss/gnss_utility.hpp"
namespace apollo {
namespace localization {
namespace local_gnss {

LeapSecond::LeapSecond() {
  size_leap_recorder_ = 0;
  Init();
}

LeapSecond::~LeapSecond() { leap_seconds_recorder_.resize(0); }

void LeapSecond::Init() {
  // leap seconds {year, month, day, hour, min, sec, utc-gpst}
  const double leaps[18][7] = {
      {2017, 1, 1, 0, 0, 0, -18}, {2015, 7, 1, 0, 0, 0, -17},
      {2012, 7, 1, 0, 0, 0, -16}, {2009, 1, 1, 0, 0, 0, -15},
      {2006, 1, 1, 0, 0, 0, -14}, {1999, 1, 1, 0, 0, 0, -13},
      {1997, 7, 1, 0, 0, 0, -12}, {1996, 1, 1, 0, 0, 0, -11},
      {1994, 7, 1, 0, 0, 0, -10}, {1993, 7, 1, 0, 0, 0, -9},
      {1992, 7, 1, 0, 0, 0, -8},  {1991, 1, 1, 0, 0, 0, -7},
      {1990, 1, 1, 0, 0, 0, -6},  {1988, 1, 1, 0, 0, 0, -5},
      {1985, 7, 1, 0, 0, 0, -4},  {1983, 7, 1, 0, 0, 0, -3},
      {1982, 7, 1, 0, 0, 0, -2},  {1981, 7, 1, 0, 0, 0, -1}};
  int leap_week = 0;
  double leap_sec = 0.0;
  for (int i = 18 - 1; i >= 0; --i) {
    leap_week = apollo::localization::local_gnss::gnss_utility::DayTime2GpsTime(
        leaps[i][0], leaps[i][1], leaps[i][2], leaps[i][3], leaps[i][4],
        leaps[i][5], &leap_sec);
    LeapRecord temp(static_cast<int>(leap_week), leap_sec, leaps[i][6]);
    leap_seconds_recorder_.push_back(temp);
  }
  return;
}

bool LeapSecond::UpdateLeapsecond(const unsigned int year,
                        const unsigned int month,
                        const unsigned int day, const unsigned int hour,
                        const unsigned int minute, const double second_s,
                        const double leap_second) {
  LeapRecord record;
  record.leap_second_s = leap_second;
  record.gps_week_num =
      apollo::localization::local_gnss::gnss_utility::DayTime2GpsTime(
          year, month, day, hour, minute, second_s, &record.gps_week_second_s);
  return UpdateLeapsecond(record);
}

bool LeapSecond::UpdateLeapsecond(const unsigned int gps_week_num,
                        const double gps_week_second_s,
                        const double leap_second) {
  LeapRecord record;
  record.leap_second_s = leap_second;
  record.gps_week_num = gps_week_num;
  record.gps_week_second_s = gps_week_second_s;
  return UpdateLeapsecond(record);
}

bool LeapSecond::UpdateLeapsecond(const LeapRecord& record) {
  const double sec_per_week = apollo::localization::local_gnss::SECOND_PER_WEEK;
  LeapRecord newest_record =
      leap_seconds_recorder_[leap_seconds_recorder_.size() - 1];
  double ton = record.gps_week_second_s + record.gps_week_num * sec_per_week;
  double tol = newest_record.gps_week_second_s +
               newest_record.gps_week_num * sec_per_week;
  if (ton <= tol) {
    return false;
  }
  if (record.leap_second_s == newest_record.leap_second_s) {
    return false;
  }
  leap_seconds_recorder_.push_back(record);
  return true;
}

double LeapSecond::GetLeapSecond(
    const unsigned int year, const unsigned int month, const unsigned int day,
    const unsigned int hour, const unsigned int minute, const double second_s) {
  double gps_week_second_s = 0.0;
  double gps_week_num =
      apollo::localization::local_gnss::gnss_utility::DayTime2GpsTime(
          year, month, day, hour, minute, second_s, &gps_week_second_s);
  return GetLeapSecond(gps_week_num, gps_week_second_s);
}

double LeapSecond::GetLeapSecond(const unsigned int gps_week_num,
                                   const double gps_week_second_s) {
  double leap_second = 0.0;
  const double sec_per_week = apollo::localization::local_gnss::SECOND_PER_WEEK;
  double tot = gps_week_num * sec_per_week + gps_week_second_s;
  unsigned int size = leap_seconds_recorder_.size();
  for (int ind = size - 1; ind >= 0; --ind) {
    double tol = leap_seconds_recorder_[ind].gps_week_second_s +
                 leap_seconds_recorder_[ind].gps_week_num * sec_per_week;
    if (tot >= tol) {
      leap_second = -1 * leap_seconds_recorder_[ind].leap_second_s;
      break;
    }
  }
  return leap_second;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
