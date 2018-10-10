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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_LEAP_SECOND_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_LEAP_SECOND_H_

#include <vector>

namespace apollo {
namespace localization {
namespace local_gnss {

class LeapSecond {
 public:
  LeapSecond();
  ~LeapSecond();

  bool UpdateLeapsecond(const unsigned int year, const unsigned int month,
              const unsigned int day, const unsigned int hour,
              const unsigned int minute, const double second_s,
              const double leap_second);

  bool UpdateLeapsecond(const unsigned int gps_week_num,
              const double gps_week_second_s,
              const double leap_second);

  double GetLeapSecond(const unsigned int year, const unsigned int month,
                         const unsigned int day, const unsigned int hour,
                         const unsigned int minute, const double second_s);

  double GetLeapSecond(const unsigned int gps_week_num,
                         const double gps_week_second_s);

  struct LeapRecord {
    // only reference to GPS time
    unsigned int gps_week_num;
    double gps_week_second_s;
    double leap_second_s;
    LeapRecord() {
      gps_week_num = 0;
      gps_week_second_s = 0.0;
      leap_second_s = 0.0;
    }
    LeapRecord(const unsigned int week_num, const double week_sec,
               const double leap_sec) {
      gps_week_num = week_num;
      gps_week_second_s = week_sec;
      leap_second_s = leap_sec;
    }
  };

 private:
  void Init();
  bool UpdateLeapsecond(const LeapRecord& record);

 private:
  unsigned int size_leap_recorder_;
  std::vector<LeapRecord> leap_seconds_recorder_;
};
}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
#endif
