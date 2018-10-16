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

#include <gtest/gtest.h>
#include "modules/localization/msf/local_gnss/gnss_positioning.h"

#include "modules/localization/msf/local_gnss/rinex_io.h"

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

namespace apollo {
namespace localization {
namespace local_gnss {

typedef apollo::drivers::gnss::EpochObservation EpochObservation;
typedef apollo::drivers::gnss::GnssEphemeris GnssEphemeris;
typedef apollo::drivers::gnss::GnssType GnssType;

class LocalGNSSTestSuite : public ::testing::Test {
 protected:
  LocalGNSSTestSuite() {}
  virtual ~LocalGNSSTestSuite() {}
  virtual void SetUp() {}
  virtual void TearDown() {}
};

// AmbiguityTracker
TEST_F(LocalGNSSTestSuite, AmbiguityTracker) {
  unsigned int max_prn = 10;
  AmbiguityTracker amb_tracker;
  int index = -1;
  for (unsigned int prn = 1; prn < max_prn; ++prn) {
    ASSERT_TRUE(amb_tracker.SetElevation(apollo::drivers::gnss::GPS_L1, prn,
                                         prn * 1.0));
    ASSERT_TRUE(
        amb_tracker.SetFixedAmb(apollo::drivers::gnss::GPS_L1, prn, prn * 1.0));
    ASSERT_FALSE(amb_tracker.GetUnfixedAmbIndex(apollo::drivers::gnss::GPS_L1,
                                                prn, &index));
  }
  const double sd_amb_res[apollo::drivers::gnss::GLO_G3] = {0.0};
  ASSERT_TRUE(amb_tracker.ReupdateReferenceAmb(sd_amb_res));

  double elv = 0.0;
  ObsKey obs_key(apollo::drivers::gnss::GPS_L1, 1);
  ASSERT_TRUE(amb_tracker.GetElevation(apollo::drivers::gnss::GPS_L1, 1, &elv));
  ASSERT_EQ(1.0, elv);
  ASSERT_FALSE(
      amb_tracker.GetElevation(apollo::drivers::gnss::GPS_L1, max_prn, &elv));
  ASSERT_EQ(1.0, amb_tracker.GetElevation(obs_key));

  double amb_val = 0.0;
  ASSERT_TRUE(
      amb_tracker.GetUnfixedAmbIndex(apollo::drivers::gnss::GPS_L1, 1, &index));
  ASSERT_EQ(0, index);
  ASSERT_EQ(max_prn - 1, amb_tracker.GetNumUnfixedAmb());

  ASSERT_TRUE(
      amb_tracker.GetFixedAmb(apollo::drivers::gnss::GPS_L1, 1, &amb_val));
  ASSERT_EQ(1.0, amb_val);
  ASSERT_FALSE(amb_tracker.GetFixedAmb(apollo::drivers::gnss::GPS_L1,
                                       max_prn + 1, &amb_val));
  ASSERT_EQ(max_prn - 1, amb_tracker.GetNumFixedAmb());

  unsigned int sat_prn = 1;
  GnssBandID sat_band_id = apollo::drivers::gnss::GPS_L1;

  ASSERT_FALSE(amb_tracker.GetUnfixedPhase(max_prn, &sat_band_id, &sat_prn));
  ASSERT_TRUE(amb_tracker.GetUnfixedPhase(max_prn - 2, &sat_band_id, &sat_prn));

  ASSERT_FALSE(amb_tracker.GetUnfixedPhase(max_prn, &obs_key));
  ASSERT_TRUE(amb_tracker.GetUnfixedPhase(max_prn - 2, &obs_key));

  obs_key.band_id = apollo::drivers::gnss::GPS_L1;
  obs_key.sat_prn = 1;
  ASSERT_FALSE(amb_tracker.GetReferenceAmb(obs_key, &amb_val));
  AmbKey ref(apollo::drivers::gnss::GPS_L1, 1);
  amb_tracker.reference_sat_.push_back(ref);
  ASSERT_TRUE(amb_tracker.GetReferenceAmb(obs_key, &amb_val));

  ASSERT_TRUE(amb_tracker.DeleteSlipAmb(apollo::drivers::gnss::GPS_L1, 1));
  ASSERT_TRUE(amb_tracker.ClearAll());

  amb_tracker.ClearHalfCycleRecorder();
  amb_tracker.AddHalfCycleRecorder(obs_key);
  ASSERT_TRUE(amb_tracker.IsHalpCycle(obs_key));
  obs_key.band_id = apollo::drivers::gnss::GPS_L2;
  ASSERT_FALSE(amb_tracker.IsHalpCycle(obs_key));

  max_prn = 10;
  AmbiguityTracker amb_tracker_del;
  unsigned int prn = 1;
  for (; prn < max_prn; ++prn) {
    ASSERT_TRUE(amb_tracker_del.SetElevation(apollo::drivers::gnss::GPS_L1, prn,
                                             prn * 5.0));
    ASSERT_TRUE(amb_tracker_del.SetFixedAmb(apollo::drivers::gnss::GPS_L1, prn,
                                            prn * 1.0));
  }
  // prn = 10
  ASSERT_TRUE(
      amb_tracker_del.SetFixedAmb(apollo::drivers::gnss::GPS_L1, 10, 10 * 1.0));
  // prn = 11
  ASSERT_TRUE(
      amb_tracker_del.SetFixedAmb(apollo::drivers::gnss::GPS_L1, 11, 11 * 1.0));

  AmbKey temp(apollo::drivers::gnss::GPS_L1, 10);
  amb_tracker_del.reference_sat_.push_back(temp);
  // the fixed apollo::drivers::gnss::GPS_L1 prn = 11, prn = 10 (as a reference
  // obs) would be deleted.
  ASSERT_TRUE(amb_tracker_del.DeleteDescendingAmb());
}

// ARMLambda
TEST_F(LocalGNSSTestSuite, ARMLambda) {
  ARMLambda ar_lambda;
  // Sign
  ASSERT_EQ(1.0, ar_lambda.Sign(5.0));
  ASSERT_EQ(-1.0, ar_lambda.Sign(-5.0));
  // Round
  ASSERT_EQ(5, ar_lambda.Round(5.3));
  ASSERT_EQ(-6.0, ar_lambda.Round(-5.8));
  // SwapData
  double a = 2.0;
  double b = 3.0;
  ar_lambda.SwapData(&a, &b);
  ASSERT_EQ(3, a);
  ASSERT_EQ(2, b);

  ARMLambda ar_mlambda;
  Eigen::MatrixXd amb_float(3, 1);
  Eigen::MatrixXd amb_cov(3, 3);

  Eigen::MatrixXd fixed_amb(3, 1);
  amb_cov.setZero();
  amb_float(0, 0) = 9.99;
  amb_float(1, 0) = 3.95;
  amb_float(2, 0) = -8.97;
  amb_cov(0, 0) = 10.0;
  amb_cov(1, 1) = 10.0;
  amb_cov(2, 2) = 10.0;

  Eigen::MatrixXd amb_fix = amb_float;
  ASSERT_TRUE(ar_mlambda.ResolveIntegerAmbiguity(amb_float, amb_cov, &amb_fix));
  ASSERT_TRUE(fabs(ar_mlambda.GetRatio() - 258.142857) < 0.1);
  ASSERT_TRUE(ar_mlambda.IsFixedSuccessfully(3.0));
  ASSERT_FALSE(ar_mlambda.IsFixedSuccessfully(300.0));

  std::vector<double> s;
  int m = 2;
  ASSERT_EQ(0, ar_mlambda.Lambda(amb_float, amb_cov, &amb_fix, &s, m));
  ASSERT_EQ(-1, ar_mlambda.Lambda(amb_float, amb_cov, &amb_fix, &s, 0));

  amb_float.resize(4, 1);
  ASSERT_FALSE(
      ar_mlambda.ResolveIntegerAmbiguity(amb_float, amb_cov, &amb_fix));
  ASSERT_EQ(-1, ar_mlambda.Lambda(amb_float, amb_cov, &amb_fix, &s, m));
  amb_cov.resize(3, 4);
  ASSERT_FALSE(
      ar_mlambda.ResolveIntegerAmbiguity(amb_float, amb_cov, &amb_fix));
  ASSERT_EQ(-1, ar_mlambda.Lambda(amb_float, amb_cov, &amb_fix, &s, m));
  amb_cov.resize(4, 3);
  ASSERT_FALSE(
      ar_mlambda.ResolveIntegerAmbiguity(amb_float, amb_cov, &amb_fix));
  ASSERT_EQ(-1, ar_mlambda.Lambda(amb_float, amb_cov, &amb_fix, &s, m));

  amb_cov.resize(3, 3);
  amb_cov.setZero();
  amb_cov(0, 0) = 0.0;
  amb_cov(1, 1) = 1.0;
  amb_cov(2, 2) = 1.0;
  // bad factorize
  Eigen::MatrixXd lower;
  std::vector<double> diag;
  ASSERT_EQ(-1, ar_mlambda.Factorize(amb_cov, &lower, &diag));
  amb_cov(0, 0) = -0.5;
  ASSERT_EQ(-1, ar_mlambda.Factorize(amb_cov, &lower, &diag));
  ASSERT_EQ(-1, ar_mlambda.Lambda(amb_float, amb_cov, &amb_fix, &s, m));

  ar_mlambda.GetGaussCdf(-3);
  ar_mlambda.GetGaussCdf(-1);
  ar_mlambda.GetGaussCdf(0);
  ar_mlambda.GetGaussCdf(0.5);
  ar_mlambda.GetGaussCdf(2.1);
  ar_mlambda.GetGaussCdf(5.0);
}

// Atmosphere
TEST_F(LocalGNSSTestSuite, Atmosphere) {
  PointThreeDim rover_coor(-2163968.048, 4384840.381, 4081633.647);
  const double elv = PI / 2.0;
  const double azm = 0.0;
  Atmosphere atm_model(rover_coor, 1915, 0.0);
  ASSERT_TRUE(fabs(atm_model.TropoDelay(elv, azm) - 2.37104) < 0.0001);
  ASSERT_TRUE(fabs(atm_model.TropoWet(elv, azm) - 0.08352) < 0.0001);
  ASSERT_TRUE(fabs(atm_model.TropoDry(elv, azm) - 2.28752) < 0.0001);
  ASSERT_TRUE(fabs(atm_model.GetMapWet(elv / 2.0, azm) - 1.41340) < 0.0001);
  ASSERT_TRUE(fabs(atm_model.GetMapDry(elv / 2.0, azm) - 1.41244) < 0.0001);
  ASSERT_TRUE(fabs(atm_model.TropoDelay(-elv / 2.0, azm) - 0.0000) < 0.0001);

  // ZtdModel
  double ztd_dry = -10.0;
  double ztd_wet = -10.0;

  atm_model.pos_blh_[2] = -1000.0;
  atm_model.ZtdModel(&ztd_dry, &ztd_wet);
  ASSERT_EQ(0, ztd_dry);
  ASSERT_EQ(0, ztd_wet);

  atm_model.pos_blh_[2] = 100000.0;
  atm_model.ZtdModel(&ztd_dry, &ztd_wet);
  ASSERT_EQ(0, ztd_dry);
  ASSERT_EQ(0, ztd_wet);

  atm_model.pos_blh_[2] = 100.0;
  atm_model.ZtdModel(&ztd_dry, &ztd_wet);

  // ZtdMapNiel
  double map_wet = 0.0;
  double map_dry = 0.0;
  atm_model.ZtdMapNiel(-0.1, 0.0, &map_wet, &map_dry);
  ASSERT_EQ(0, map_wet);
  ASSERT_EQ(0, map_dry);
  atm_model.ZtdMapNiel(0.0, 0.0, &map_wet, &map_dry);
  atm_model.ZtdMapNiel(3.1415926 / 4, 0.0, &map_wet, &map_dry);

  // InterploateCoeff
  double lat = 10.0;
  const double coef[15] = {1.0};
  atm_model.InterploateCoeff(coef, lat);
  lat = 75.0;
  atm_model.InterploateCoeff(coef, lat);

  // Gpstime2Dof
  double week_second_s = 0.0;
  int week_num = 0.0;
  week_num =
      gnss_utility::DayTime2GpsTime(2017, 1, 1, 0, 0, 0.0, &week_second_s);
  atm_model.Gpstime2Dof(week_num, week_second_s);
  week_num =
      gnss_utility::DayTime2GpsTime(2017, 2, 1, 0, 0, 0.0, &week_second_s);
  atm_model.Gpstime2Dof(week_num, week_second_s);
  week_num =
      gnss_utility::DayTime2GpsTime(2016, 2, 1, 0, 0, 0.0, &week_second_s);
  atm_model.Gpstime2Dof(week_num, week_second_s);
  week_num =
      gnss_utility::DayTime2GpsTime(2016, 2, 29, 0, 0, 0.0, &week_second_s);
  atm_model.Gpstime2Dof(week_num, week_second_s);
  week_num =
      gnss_utility::DayTime2GpsTime(1979, 2, 26, 0, 0, 0.0, &week_second_s);
  atm_model.Gpstime2Dof(week_num, week_second_s);
}

// gnss_constants
TEST_F(LocalGNSSTestSuite, gnss_constants) {
  GpsEllipsoidParam gps_eps;
  BdsEllipsoidParam bds_eps;
  GloEllipsoidParam glo_eps;
  GalEllipsoidParam gal_eps;
}

// gnss_utility
TEST_F(LocalGNSSTestSuite, gnss_utility) {
  ObsKey obs_key1(apollo::drivers::gnss::GPS_L1, 1);
  ObsKey obs_key2(apollo::drivers::gnss::GPS_L1, 1);
  ObsKey obs_key3(apollo::drivers::gnss::GPS_L1, 2);
  ObsKey obs_key4(apollo::drivers::gnss::GPS_L2, 1);
  ASSERT_TRUE(obs_key1 < obs_key4);
  ASSERT_FALSE(obs_key4 < obs_key1);
  ASSERT_FALSE(obs_key1 < obs_key1);
  ASSERT_FALSE(obs_key3 < obs_key1);
  ASSERT_TRUE(obs_key1 == obs_key2);
  ASSERT_FALSE(obs_key1 == obs_key3);

  // EphKey
  EphKey eph_key1(apollo::drivers::gnss::GPS_SYS, 1, 0.0);
  EphKey eph_key2(apollo::drivers::gnss::GPS_SYS, 1, 0.0);
  EphKey eph_key3(apollo::drivers::gnss::GPS_SYS, 1, 1.0);
  EphKey eph_key4(apollo::drivers::gnss::GPS_SYS, 2, 1.0);
  EphKey eph_key5(apollo::drivers::gnss::BDS_SYS, 1, 1.0);
  ASSERT_TRUE(eph_key1 < eph_key5);
  ASSERT_FALSE(eph_key5 < eph_key1);
  ASSERT_FALSE(eph_key5 < eph_key5);

  ASSERT_TRUE(eph_key1 < eph_key4);
  ASSERT_FALSE(eph_key4 < eph_key1);
  ASSERT_FALSE(eph_key4 < eph_key4);

  ASSERT_TRUE(eph_key1 < eph_key3);
  ASSERT_FALSE(eph_key3 < eph_key1);

  ASSERT_TRUE(eph_key1 == eph_key2);
  ASSERT_FALSE(eph_key1 == eph_key3);

  // GpsTime2DayTime
  int year = 1981;
  int month = 1;
  int day = 6;
  int hour = 0;
  int minute = 0;
  double sec = 0;
  int week_num = 0;
  double week_sec = 0;
  week_num = gnss_utility::DayTime2GpsTime(1980, month, day, hour, minute, sec,
                                           &week_sec);

  week_num = gnss_utility::DayTime2GpsTime(year, month, day, hour, minute, sec,
                                           &week_sec);
  ASSERT_EQ(52, week_num);
  ASSERT_EQ(172800, week_sec);

  week_num = gnss_utility::DayTime2GpsTime(2017, 1, 1, 0, 0, 0, &week_sec);
  week_num = gnss_utility::DayTime2GpsTime(2017, 2, 1, 0, 0, 0, &week_sec);
  week_num = gnss_utility::DayTime2GpsTime(2016, 2, 1, 0, 0, 0, &week_sec);
  gnss_utility::GpsTime2DayTime(week_num, week_sec, &year, &month, &day, &hour,
                                &minute, &sec);
  week_num = gnss_utility::DayTime2GpsTime(1980, 1, 1, 0, 0, 0, &week_sec);
  // Morning January 6, 1980
  week_num = 0;
  week_sec = 0;
  gnss_utility::GpsTime2DayTime(week_num, week_sec, &year, &month, &day, &hour,
                                &minute, &sec);
  ASSERT_EQ(1980, year);
  ASSERT_EQ(1, month);
  ASSERT_EQ(6, day);
  ASSERT_EQ(0, hour);
  ASSERT_EQ(0, minute);
  ASSERT_EQ(0, sec);

  std::vector<int> index_g;
  index_g.resize(0);
  for (int i = 0; i < 5; ++i) {
    index_g.push_back(i);
  }
  ASSERT_EQ(-1, gnss_utility::GetIndexInVector(7, index_g));
  ASSERT_EQ(0, gnss_utility::GetIndexInVector(0, index_g));

  double xyz[3] = {0.0};
  double blh[3] = {1.5, 1.5, 30.0};

  gnss_utility::llh2xyz(blh[0], blh[1], blh[2], xyz);
  ASSERT_TRUE(fabs(xyz[0] - (32021.5876741)) < 0.00001);
  ASSERT_TRUE(fabs(xyz[1] - (451549.8551679)) < 0.00001);
  ASSERT_TRUE(fabs(xyz[2] - (6340751.5769508)) < 0.00001);
  gnss_utility::xyz2llh(xyz, &blh[0], &blh[1], &blh[2]);
  ASSERT_TRUE(fabs(blh[0] - 1.5) < 0.000000001);
  ASSERT_TRUE(fabs(blh[1] - 1.5) < 0.000000001);
  ASSERT_TRUE(fabs(blh[2] - 30.0) < 0.0001);

  double enu[3] = {1.0, 1.0, 1.0};
  double dxyz[3] = {1.0, 1.0, 1.0};
  gnss_utility::dxyz2enu(dxyz, blh[0], blh[1], enu);
  ASSERT_TRUE(fabs(enu[0] - (-0.9267578)) < 0.00001);
  ASSERT_TRUE(fabs(enu[1] - (-0.9948191)) < 0.00001);
  ASSERT_TRUE(fabs(enu[2] - (+1.0730587)) < 0.00001);
  gnss_utility::enu2xyz(enu, blh[0], blh[1], dxyz);

  PointThreeDim point_dxyz(0, 0, 0);
  gnss_utility::enu2xyz(enu, blh[0], blh[1], &point_dxyz);

  ASSERT_TRUE(fabs(dxyz[0] - 1.0) < 0.00001);
  ASSERT_TRUE(fabs(dxyz[1] - 1.0) < 0.00001);
  ASSERT_TRUE(fabs(dxyz[2] - 1.0) < 0.00001);

  PointThreeDim src(0, 0, 0);
  PointThreeDim dest(3, 4, 0);
  ASSERT_EQ(5.0, gnss_utility::GetDistance(src, dest));

  ASSERT_EQ(1.0, gnss_utility::sign(0.0));
  ASSERT_EQ(1.0, gnss_utility::sign(1.0));
  ASSERT_EQ(-1.0, gnss_utility::sign(-1.0));

  Eigen::MatrixXd m(1, 1);
  m(0, 0) = 1.0;
  gnss_utility::RoundMatrix(m);
  gnss_utility::DcmBody2Navi(0.0, 0.0, 0.0);

  // GetEleAzm
  double elv_deg = 0;
  double azm_deg = 0;
  // azm = 0
  dest.x = 0;
  dest.y = 1.0;
  gnss_utility::GetEleAzm(src, dest, &elv_deg, &azm_deg);
  // azm = 45
  dest.x = 1.0;
  dest.y = 1.0;
  gnss_utility::GetEleAzm(src, dest, &elv_deg, &azm_deg);
  // azm = 90
  dest.x = 1.0;
  dest.y = 0.0;
  gnss_utility::GetEleAzm(src, dest, &elv_deg, &azm_deg);
  // azm = 135
  dest.x = 1.0;
  dest.y = -1.0;
  gnss_utility::GetEleAzm(src, dest, &elv_deg, &azm_deg);
  // azm = 180
  dest.x = 0;
  dest.y = -1.0;
  gnss_utility::GetEleAzm(src, dest, &elv_deg, &azm_deg);
  // azm = 225
  dest.x = -1.0;
  dest.y = -1.0;
  gnss_utility::GetEleAzm(src, dest, &elv_deg, &azm_deg);
  // azm = 270
  dest.x = -1.0;
  dest.y = 0.0;
  gnss_utility::GetEleAzm(src, dest, &elv_deg, &azm_deg);
  // azm = 315
  dest.x = -1.0;
  dest.y = 1.0;
  gnss_utility::GetEleAzm(src, dest, &elv_deg, &azm_deg);

  // xyz2llh
  xyz[1] = 0;
  xyz[0] = 1.0;
  gnss_utility::xyz2llh(xyz, &blh[0], &blh[1], &blh[2]);

  xyz[1] = 0;
  xyz[0] = -1.0;
  gnss_utility::xyz2llh(xyz, &blh[0], &blh[1], &blh[2]);

  // SatelliteInfor
  SatelliteInfor temp1;
  SatelliteInfor temp2;
  temp1.sat_sys = apollo::drivers::gnss::GPS_SYS;
  temp1.sat_prn = 1;
  temp2.sat_sys = apollo::drivers::gnss::BDS_SYS;
  temp2.sat_prn = 2;
  ASSERT_FALSE(temp1.IsSameSatellite(temp2));
  temp2.sat_sys = apollo::drivers::gnss::GPS_SYS;
  ASSERT_FALSE(temp1.IsSameSatellite(temp2));
  temp2.sat_prn = 1;
  ASSERT_TRUE(temp1.IsSameSatellite(temp2));
}

// LeapSecond
TEST_F(LocalGNSSTestSuite, LeapSecond) {
  LeapSecond leap_sec;
  ASSERT_EQ(18, leap_sec.GetLeapSecond(1955, 518400));
  ASSERT_FALSE(leap_sec.UpdateLeapsecond(2015, 7, 1, 0, 0, 0, -17));
  ASSERT_FALSE(leap_sec.UpdateLeapsecond(2017, 1, 1, 0, 0, 0, -18));
  ASSERT_FALSE(leap_sec.UpdateLeapsecond(2017, 7, 1, 0, 0, 0, -18));
  ASSERT_TRUE(leap_sec.UpdateLeapsecond(2018, 7, 1, 0, 0, 0, -19));
  // 2015 07 01
  ASSERT_FALSE(leap_sec.UpdateLeapsecond(1851, 259200.000, -17));
  // 2017 07 01
  ASSERT_FALSE(leap_sec.UpdateLeapsecond(1955, 518400.000, -18));
  // 2019 07 01
  ASSERT_TRUE(leap_sec.UpdateLeapsecond(2060, 86400.000, -20));
}

// SatelliteInterface
TEST_F(LocalGNSSTestSuite, SatelliteInterface) {
  // SetEphMaxsize
  SatelliteInterface all_eph(12 * 2, 24 * 2, 48 * 2);
  ASSERT_TRUE(all_eph.SetEphMaxsize(24, apollo::drivers::gnss::GPS_SYS));
  ASSERT_TRUE(all_eph.SetEphMaxsize(48, apollo::drivers::gnss::BDS_SYS));
  ASSERT_TRUE(all_eph.SetEphMaxsize(96, apollo::drivers::gnss::GLO_SYS));
  ASSERT_FALSE(all_eph.SetEphMaxsize(100, apollo::drivers::gnss::SYS_UNKNOWN));
  // Mock test GNSS data
  GnssEphemeris gnss_orbit;

  gnss_orbit.Clear();
  gnss_orbit.set_gnss_type(apollo::drivers::gnss::SYS_UNKNOWN);
  ASSERT_TRUE(all_eph.CheckDuplicatedEph(gnss_orbit));
  ASSERT_FALSE(all_eph.SaveEphemeris(gnss_orbit));

  gnss_orbit.set_gnss_type(apollo::drivers::gnss::GLO_SYS);
  gnss_orbit.mutable_glonass_orbit()->set_week_num(1968);
  gnss_orbit.mutable_glonass_orbit()->set_toe(464557);
  gnss_orbit.mutable_glonass_orbit()->set_week_second_s(464557);
  gnss_orbit.mutable_glonass_orbit()->set_slot_prn(1);
  ASSERT_FALSE(all_eph.CheckDuplicatedEph(gnss_orbit));
  ASSERT_FALSE(all_eph.SaveEphemeris(gnss_orbit));

  ASSERT_TRUE(all_eph.CheckDuplicatedEph(gnss_orbit));

  gnss_orbit.set_gnss_type(apollo::drivers::gnss::GPS_SYS);
  gnss_orbit.mutable_keppler_orbit()->set_week_num(1968);
  gnss_orbit.mutable_keppler_orbit()->set_toe(464557);
  gnss_orbit.mutable_keppler_orbit()->set_sat_prn(1);
  ASSERT_TRUE(all_eph.SaveEphemeris(gnss_orbit));

  gnss_orbit.mutable_keppler_orbit()->set_sat_prn(2);
  ASSERT_FALSE(all_eph.CheckDuplicatedEph(gnss_orbit));
  gnss_orbit.mutable_keppler_orbit()->set_sat_prn(3);
  ASSERT_FALSE(all_eph.CheckDuplicatedEph(gnss_orbit));

  gnss_orbit.set_gnss_type(apollo::drivers::gnss::BDS_SYS);
  ASSERT_TRUE(all_eph.SaveEphemeris(gnss_orbit));

  int glo_fre_num = 0;
  ASSERT_EQ(
      0.190293672798,
      all_eph.GetBandLength(1, apollo::drivers::gnss::GPS_L1, &glo_fre_num));
  ASSERT_EQ(
      0.244210213425,
      all_eph.GetBandLength(1, apollo::drivers::gnss::GPS_L2, &glo_fre_num));
  ASSERT_EQ(0.254828049, all_eph.GetBandLength(1, apollo::drivers::gnss::GPS_L5,
                                               &glo_fre_num));
  ASSERT_EQ(
      0.192039486310276,
      all_eph.GetBandLength(1, apollo::drivers::gnss::BDS_B1, &glo_fre_num));
  ASSERT_EQ(0.24834937, all_eph.GetBandLength(1, apollo::drivers::gnss::BDS_B2,
                                              &glo_fre_num));
  ASSERT_EQ(
      0.236332464604421,
      all_eph.GetBandLength(1, apollo::drivers::gnss::BDS_B3, &glo_fre_num));

  ASSERT_TRUE(fabs(all_eph.GetBandLength(1, apollo::drivers::gnss::GLO_G1,
                                         &glo_fre_num) -
                   0.187071) < 0.0001);
  ASSERT_TRUE(fabs(all_eph.GetBandLength(1, apollo::drivers::gnss::GLO_G2,
                                         &glo_fre_num) -
                   0.240519) < 0.0001);
}

TEST_F(LocalGNSSTestSuite, GPSPvt) {
  GPSPvt pvt_gps;
  double tk = 604800;
  pvt_gps.CheckValidReferIOD(&tk);
  ASSERT_EQ(0, tk);
  tk = 302399;
  pvt_gps.CheckValidReferIOD(&tk);
  ASSERT_EQ(302399, tk);
  tk = -604800;
  pvt_gps.CheckValidReferIOD(&tk);
  ASSERT_EQ(0, tk);
  const double mk = 2.0935748619413612;
  const double eph_e = 0.0052386270836000002;
  double ek = 0;
  ek = pvt_gps.ComputeEhpEk(mk, eph_e, 1);
  ek = pvt_gps.ComputeEhpEk(mk, eph_e, 5);
  ek = pvt_gps.ComputeEhpEk(mk, eph_e, 10);
  ek = ek + 1;

  // 1) SaveEph
  KepplerOrbit current_eph;
  current_eph.Clear();
  current_eph.set_sat_prn(0);
  ASSERT_FALSE(pvt_gps.SaveEph(current_eph));
  current_eph.set_sat_prn(32 + 1);
  ASSERT_FALSE(pvt_gps.SaveEph(current_eph));
  pvt_gps.max_ephsize_gnss_ = 1;
  current_eph.set_sat_prn(5);
  pvt_gps.SaveEph(current_eph);
  pvt_gps.SaveEph(current_eph);
  pvt_gps.SaveEph(current_eph);

  // 2)SortCorrectEphemeris
  pvt_gps.Initialize();
  ASSERT_FALSE(pvt_gps.SortCorrectEphemeris(35, 0.0, &current_eph));
  ASSERT_FALSE(pvt_gps.SortCorrectEphemeris(1, 0.0, &current_eph));

  current_eph.set_sat_prn(1);
  pvt_gps.SaveEph(current_eph);
  ASSERT_TRUE(pvt_gps.SortCorrectEphemeris(1, 0.0, &current_eph));
  pvt_gps.current_eph_index_.resize(0);
  ASSERT_FALSE(pvt_gps.SortCorrectEphemeris(10, 0.0, &current_eph));
  pvt_gps.current_eph_index_.resize(11);
  pvt_gps._gnss_eph.resize(10);
  ASSERT_FALSE(pvt_gps.SortCorrectEphemeris(10, 0.0, &current_eph));

  // 3)GetPosVelClock
  PointThreeDim pos;
  PointThreeDim vel;
  double clk_bias;
  double clk_drift;
  double eph_toe = 0.0;
  GPSPvt p_pvt_gps;
  current_eph.Clear();
  current_eph.set_sat_prn(1);
  current_eph.set_toe(1.0);
  current_eph.set_week_num(0);
  current_eph.set_gnss_type(apollo::drivers::gnss::GPS_SYS);
  current_eph.set_health(1);
  p_pvt_gps.SaveEph(current_eph);

  ASSERT_FALSE(p_pvt_gps.GetPosVelClock(10, 0, 0, 0, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
  eph_toe = -0.1;
  ASSERT_FALSE(p_pvt_gps.GetPosVelClock(10, 0, 0, 0, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
  ASSERT_FALSE(p_pvt_gps.GetPosVelClock(1, 0, 1.0, 0.078, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
}

TEST_F(LocalGNSSTestSuite, BeidouPvt) {
  BeidouPvt pvt_bds;
  KepplerOrbit current_eph;

  PointThreeDim pos;
  PointThreeDim vel;
  double clk_bias;
  double clk_drift;
  double eph_toe = 0.0;
  GPSPvt p_pvt_bds;
  current_eph.Clear();
  current_eph.set_sat_prn(1);
  current_eph.set_toe(1.0);
  current_eph.set_week_num(0);
  current_eph.set_gnss_type(apollo::drivers::gnss::BDS_SYS);
  current_eph.set_health(1);
  p_pvt_bds.SaveEph(current_eph);

  ASSERT_FALSE(p_pvt_bds.GetPosVelClock(10, 0, 0, 0, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
  eph_toe = -0.1;
  ASSERT_FALSE(p_pvt_bds.GetPosVelClock(10, 0, 0, 0, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
  ASSERT_FALSE(p_pvt_bds.GetPosVelClock(1, 0, 1.0, 0.078, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
  current_eph.set_iode(1.0);
  current_eph.set_crs(-4.241875000000e+02);
  current_eph.set_deltan(4.504830501415e-09);
  current_eph.set_m0(-3.208559911342e-01);

  current_eph.set_cuc(-1.383945345879e-05);
  current_eph.set_e(3.910805098712e-04);
  current_eph.set_cus(1.237168908119e-05);
  current_eph.set_roota(6.493485338211e+03);

  current_eph.set_toe(1.728000000000e+05);
  current_eph.set_toc(1.728000000000e+05);
  current_eph.set_cic(1.206062734127e-07);
  current_eph.set_omega0(-2.940159690390e+00);
  current_eph.set_cis(-3.492459654808e-08);

  current_eph.set_i0(6.494347589931e-02);
  current_eph.set_crc(-3.733281250000e+02);
  current_eph.set_omega(-5.439893968962e-01);
  current_eph.set_omegadot(-3.480859277475e-09);

  current_eph.set_idot(-4.360895934534e-10);

  current_eph.set_sat_prn(1);
  ASSERT_TRUE(
      p_pvt_bds.GetPositionVelocity(1.728e+050, current_eph, &pos, &vel));

  current_eph.set_sat_prn(5);
  ASSERT_TRUE(
      p_pvt_bds.GetPositionVelocity(1.728e+050, current_eph, &pos, &vel));
}

TEST_F(LocalGNSSTestSuite, GlonassPvt) {
  GlonassPvt pvt_glo;
  GlonassOrbit current_eph;
  PointThreeDim pos;
  PointThreeDim vel;
  double clk_bias;
  double clk_drift;
  double eph_toe = 0.0;
  GlonassPvt p_pvt_glo;
  current_eph.Clear();
  current_eph.set_slot_prn(1);
  current_eph.set_toe(1.0);
  current_eph.set_week_num(0);
  current_eph.set_gnss_type(apollo::drivers::gnss::GLO_SYS);
  current_eph.set_health(1);
  p_pvt_glo.SaveEph(current_eph);

  ASSERT_FALSE(p_pvt_glo.GetPosVelClock(10, 0, 0, 0, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
  eph_toe = -0.1;
  ASSERT_FALSE(p_pvt_glo.GetPosVelClock(10, 0, 0, 0, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
  ASSERT_FALSE(p_pvt_glo.GetPosVelClock(1, 0, 1.0, 0.078, &pos, &vel, &clk_bias,
                                        &clk_drift, &eph_toe));
  p_pvt_glo._gnss_eph.clear();
  p_pvt_glo.Initialize();
  current_eph.set_gnss_type(apollo::drivers::gnss::GLO_SYS);
  current_eph.set_health(0);
  current_eph.set_slot_prn(1);

  int week_num = 0;
  double gps_second_s = 0;
  week_num =
      gnss_utility::DayTime2GpsTime(2017, 10, 24, 0, 15, 00, &gps_second_s);
  double observe_time = 604800 * week_num + gps_second_s;

  current_eph.set_toe(gps_second_s);
  current_eph.set_week_second_s(gps_second_s);
  current_eph.set_week_num(week_num);

  current_eph.set_clock_offset(1.702923327684e-05);
  current_eph.set_clock_drift(0);
  current_eph.set_tk(1.728000000000e+05);

  current_eph.set_position_x(2.042149462891e+04);
  current_eph.set_velocity_x(-1.531925201416e+00);
  current_eph.set_accelerate_x(9.313225746155e-10);
  current_eph.set_health(0);

  current_eph.set_position_y(-7.338703613281e+03);
  current_eph.set_velocity_y(1.018904685974e+00);
  current_eph.set_accelerate_y(9.313225746155e-10);
  current_eph.set_frequency_no(1);

  current_eph.set_position_z(-1.342316845703e+04);
  current_eph.set_velocity_z(-2.886641502380e+00);
  current_eph.set_accelerate_z(0);
  current_eph.set_infor_age(1.862645149231e-09);

  ASSERT_TRUE(p_pvt_glo.GetPositionVelocity(observe_time + 1.0, current_eph,
                                            &pos, &vel));
  ASSERT_TRUE(p_pvt_glo.GetPositionVelocity(observe_time - 1.0, current_eph,
                                            &pos, &vel));
}

TEST_F(LocalGNSSTestSuite, PreProcessor) {
  PreProcessor pre_processor;
  SlipKey temp(apollo::drivers::gnss::GPS_L1, 1);
  double CheckSlipValue = 1.0;
  pre_processor.Reset();
  pre_processor.slip_recorder_.insert(
      std::map<SlipKey, double>::value_type(temp, 1.0));
  ASSERT_TRUE(pre_processor.CheckSlipValue(apollo::drivers::gnss::GPS_L1, 1,
                                           &CheckSlipValue));
  ASSERT_FALSE(pre_processor.CheckSlipValue(apollo::drivers::gnss::GPS_L1, 2,
                                            &CheckSlipValue));
  pre_processor.Reset();

  BandObservation band_obs1;
  BandObservation band_obs2;
  band_obs1.Clear();
  band_obs2.Clear();
  band_obs1.set_band_id(apollo::drivers::gnss::GPS_L1);
  band_obs2.set_band_id(apollo::drivers::gnss::GPS_L2);
  ASSERT_EQ(-1, pre_processor.CheckBandPhaseObs(band_obs1, band_obs2));

  band_obs2.set_band_id(apollo::drivers::gnss::GPS_L1);
  pre_processor.band_to_solve_.resize(0);
  ASSERT_EQ(-1, pre_processor.CheckBandPhaseObs(band_obs1, band_obs2));

  pre_processor.band_to_solve_.push_back(apollo::drivers::gnss::GPS_L1);
  band_obs1.set_loss_lock_index(2);

  band_obs1.set_carrier_phase(0.0);
  ASSERT_EQ(-1, pre_processor.CheckBandPhaseObs(band_obs1, band_obs2));

  band_obs1.set_carrier_phase(1.0);
  band_obs2.set_carrier_phase(0.0);
  ASSERT_EQ(-1, pre_processor.CheckBandPhaseObs(band_obs1, band_obs2));

  band_obs2.set_carrier_phase(1.0);
  ASSERT_EQ(0, pre_processor.CheckBandPhaseObs(band_obs1, band_obs2));

  std::vector<GnssBandID> bands;
  bands.resize(0);
  ASSERT_FALSE(pre_processor.SetResolvedBands(bands));
  bands.push_back(apollo::drivers::gnss::GPS_L1);
  bands.push_back(apollo::drivers::gnss::GPS_L2);
  bands.push_back(apollo::drivers::gnss::BDS_B1);
  bands.push_back(apollo::drivers::gnss::BDS_B2);
  bands.push_back(apollo::drivers::gnss::GLO_G1);
  bands.push_back(apollo::drivers::gnss::GLO_G2);
  bands.push_back(apollo::drivers::gnss::BAND_UNKNOWN);
  ASSERT_TRUE(pre_processor.SetResolvedBands(bands));
  pre_processor.Initialize();

  pre_processor.global_ephemeris_ptr_ = NULL;
  SatelliteInfor last_sat;
  last_sat.sat_sys = apollo::drivers::gnss::GPS_SYS;
  last_sat.sat_prn = 1;
  last_sat.gnss_week = 1965;
  last_sat.time_signal_transmitted = 360000;
  last_sat.time_travles = 0.070;
  ASSERT_FALSE(pre_processor.UpdateSatInforWithNewEph(&last_sat));

  std::vector<GnssType> related_gnss_type;
  apollo::localization::local_gnss::PointThreeDim pos;
  apollo::localization::local_gnss::EpochObservation newobs;
  std::vector<SatelliteInfor> sat_list;
  ASSERT_FALSE(pre_processor.DetectBasedOnGeometry(related_gnss_type, pos,
                                                   newobs, sat_list));

  pre_processor.last_obs_time_ = 100.0;
  newobs.set_gnss_week(0);
  newobs.set_gnss_second_s(100.01);
  ASSERT_FALSE(pre_processor.DetectBasedOnGeometry(related_gnss_type, pos,
                                                   newobs, sat_list));

  newobs.set_gnss_second_s(106.01);
  ASSERT_FALSE(pre_processor.DetectBasedOnGeometry(related_gnss_type, pos,
                                                   newobs, sat_list));

  newobs.set_gnss_second_s(101.0);
  ASSERT_FALSE(pre_processor.DetectBasedOnGeometry(related_gnss_type, pos,
                                                   newobs, sat_list));
  pre_processor.ReInitialize();
  pre_processor.EnablePrint(false);
}

// gnss_positioning setting part
TEST_F(LocalGNSSTestSuite, gnss_settings) {
  const PositionOption option = RTK;
  GnssPntSolver gnss_locator;
  gnss_locator.SetPositionOption(option);
  ASSERT_EQ(option, gnss_locator.GetPositionOption());

  const double elv_cut_off = 15.0;
  gnss_locator.SetElevationCutoffDeg(elv_cut_off);
  ASSERT_EQ(elv_cut_off, gnss_locator.GetElevationCutoffDeg());

  const unsigned int baser_buff_size = 10;
  gnss_locator.SetBaserBufferSize(baser_buff_size);
  ASSERT_EQ(baser_buff_size, gnss_locator.GetBaserBufferSize());

  const bool half_cycle_ar = true;
  gnss_locator.EnableHalfCycleAr(half_cycle_ar);
  ASSERT_EQ(half_cycle_ar, gnss_locator.GetHalfCycleAr());

  const double glo_ifb = 0.01;
  gnss_locator.SetGlonassIfb(glo_ifb);
  ASSERT_EQ(glo_ifb, gnss_locator.GetGlonassIfb());

  const bool external_predict = true;
  gnss_locator.SetEnableExternalPrediction(external_predict);
  ASSERT_EQ(external_predict, gnss_locator.GetEnableExternalPrediction());

  const unsigned int gps_week_num = 1945;
  const double gps_week_second_s = 108857.000;
  ASSERT_EQ(18.0, gnss_locator.GetLeapSecond(gps_week_num, gps_week_second_s));
}

TEST_F(LocalGNSSTestSuite, GnssPntSolver) {
  GnssPntSolver pnt_solver;
  pnt_solver.AddSolvedBand(apollo::drivers::gnss::GPS_L1);
  pnt_solver.AddSolvedBand(apollo::drivers::gnss::GPS_L1);
  pnt_solver.AddSolvedBand(apollo::drivers::gnss::GPS_L2);
  pnt_solver.AddSolvedBand(apollo::drivers::gnss::GPS_L5);

  std::vector<GnssType> related_gnss_type;
  related_gnss_type.resize(0);
  ASSERT_FALSE(pnt_solver.IsGlonassExisted(related_gnss_type));
  related_gnss_type.push_back(apollo::drivers::gnss::GPS_SYS);
  related_gnss_type.push_back(apollo::drivers::gnss::GLO_SYS);
  ASSERT_TRUE(pnt_solver.IsGlonassExisted(related_gnss_type));

  pnt_solver.WeightScaleOnGnssSystem(apollo::drivers::gnss::GPS_SYS);
  pnt_solver.WeightScaleOnGnssSystem(apollo::drivers::gnss::BDS_SYS);
  pnt_solver.WeightScaleOnGnssSystem(apollo::drivers::gnss::GLO_SYS);
  pnt_solver.WeightScaleOnGnssSystem(apollo::drivers::gnss::SYS_UNKNOWN);

  pnt_solver.CheckPossibleHalfSlip(1);
  pnt_solver.CheckPossibleHalfSlip(2);

  SatelliteObservation one_sat;
  one_sat.Clear();
  BandObservation band_obs;
  band_obs.Clear();
  *one_sat.add_band_obs() = band_obs;
  one_sat.set_band_obs_num(one_sat.band_obs_size());
  ASSERT_FALSE(pnt_solver.ExcludeSemiCycleSat(one_sat));

  band_obs.set_loss_lock_index(2);
  *one_sat.mutable_band_obs(0) = band_obs;
  ASSERT_TRUE(pnt_solver.ExcludeSemiCycleSat(one_sat));

  band_obs.set_loss_lock_index(0);
  *one_sat.mutable_band_obs(0) = band_obs;
  ASSERT_FALSE(pnt_solver.ExcludeSemiCycleSat(one_sat));

  const double elv_cutoff = 15;
  std::vector<SatelliteInfor> sat_vector;
  sat_vector.resize(0);
  SatelliteInfor sat;

  sat.sat_sys = apollo::drivers::gnss::GPS_SYS;
  sat.sat_prn = 1;

  sat.elevation = 20;
  sat.multi_path = false;
  sat_vector.push_back(sat);

  sat.elevation = 20;
  sat.multi_path = true;
  sat_vector.push_back(sat);

  sat.elevation = 10;
  sat.multi_path = false;
  sat_vector.push_back(sat);

  sat.elevation = 10;
  sat.multi_path = true;
  sat_vector.push_back(sat);
  ASSERT_TRUE(pnt_solver.DeleteElvCutoffSat(elv_cutoff, &sat_vector));

  pnt_solver.band_to_solve_.resize(0);
  ASSERT_FALSE(pnt_solver.GetGnssSystemToSolve());
  pnt_solver.band_to_solve_.push_back(apollo::drivers::gnss::GPS_L1);
  pnt_solver.band_to_solve_.push_back(apollo::drivers::gnss::GPS_L2);
  pnt_solver.band_to_solve_.push_back(apollo::drivers::gnss::BDS_B1);
  pnt_solver.band_to_solve_.push_back(apollo::drivers::gnss::BDS_B2);
  pnt_solver.band_to_solve_.push_back(apollo::drivers::gnss::GLO_G1);
  pnt_solver.band_to_solve_.push_back(apollo::drivers::gnss::GLO_G2);
  pnt_solver.band_to_solve_.push_back(apollo::drivers::gnss::BAND_UNKNOWN);
  ASSERT_TRUE(pnt_solver.GetGnssSystemToSolve());

  BandObservation band_obs1;
  BandObservation band_obs2;
  band_obs1.Clear();
  band_obs2.Clear();
  band_obs1.set_band_id(apollo::drivers::gnss::GPS_L1);
  band_obs2.set_band_id(apollo::drivers::gnss::GPS_L2);
  ASSERT_EQ(-1, pnt_solver.IsValidBandObs(band_obs1, band_obs2));

  band_obs2.set_band_id(apollo::drivers::gnss::GPS_L1);
  pnt_solver.band_to_solve_.resize(0);
  ASSERT_EQ(-1, pnt_solver.IsValidBandObs(band_obs1, band_obs2));

  pnt_solver.band_to_solve_.push_back(apollo::drivers::gnss::GPS_L1);
  ASSERT_EQ(0, pnt_solver.IsValidBandObs(band_obs1, band_obs2));

  pnt_solver.b_rtkfixed_ = false;

  pnt_solver.b_rtkfixed_ = true;
  ASSERT_TRUE(pnt_solver.FixJudge(5.0, 1.5, 2.0, 0.01));

  pnt_solver.b_rtkfixed_ = false;
  ASSERT_FALSE(pnt_solver.FixJudge(1.0, 1.5, 10.1, 0.01));
  ASSERT_FALSE(pnt_solver.FixJudge(1.0, 1.5, 10.0, 0.01));
  ASSERT_FALSE(pnt_solver.FixJudge(1.0, 1.5, 10.0, 0.03));
  ASSERT_FALSE(pnt_solver.FixJudge(1.0, 1.5, 2.0, 0.03));

  ASSERT_FALSE(pnt_solver.FixJudge(1.3, 2.0, 2.0, 0.01));
  ASSERT_FALSE(pnt_solver.FixJudge(5.0, 1.5, 2.0, 0.01));
  ASSERT_TRUE(pnt_solver.FixJudge(5.0, 1.5, 2.0, 0.01));

  pnt_solver.b_rtkfixed_ = false;
  ASSERT_FALSE(pnt_solver.FixJudge(5.0, 1.5, 2.0, 0.01, 3));
  ASSERT_FALSE(pnt_solver.FixJudge(5.0, 1.5, 2.0, 0.01, 3));
  ASSERT_TRUE(pnt_solver.FixJudge(5.0, 1.5, 2.0, 0.01, 3));

  ASSERT_FALSE(pnt_solver.FixJudgeCombined(1.0, 1.5, 2.0, 0.03, 0.1));
  pnt_solver.FixJudgeCombined(5.0, 1.5, 2.0, 0.01, 0.01);

  // 13)IsEnoughPhaseObs
  ASSERT_FALSE(pnt_solver.IsEnoughPhaseObs(2, 2, 2));
  ASSERT_FALSE(pnt_solver.IsEnoughPhaseObs(20, 2, 1));
  ASSERT_TRUE(pnt_solver.IsEnoughPhaseObs(20, 2, 2));

  // 14) ResolveAmbiguity
  Eigen::MatrixXd float_dd(2, 2);
  Eigen::MatrixXd float_dd_cvc(2, 2);
  Eigen::MatrixXd inter_dd;
  ASSERT_EQ(-1, pnt_solver.ResolveAmbiguity(float_dd, float_dd_cvc, &inter_dd));
  float_dd.resize(10, 1);
  float_dd.resize(9, 9);
  ASSERT_EQ(-1, pnt_solver.ResolveAmbiguity(float_dd, float_dd_cvc, &inter_dd));

  // 15)IsToughNewSat + SeedToughNewSat + SeedToughSatGroup
  pnt_solver.tough_fixing_pool_.clear();
  pnt_solver.amb_tracker_.unresolved_phase_.clear();
  ObsKey temp(apollo::drivers::gnss::GPS_L1, 1);
  for (unsigned int prn = 1; prn < 5; ++prn) {
    temp.sat_prn = prn;
    pnt_solver.SeedToughNewSat(temp, 1.0 * prn);
    pnt_solver.amb_tracker_.unresolved_phase_.push_back(temp);
  }
  pnt_solver.SeedToughNewSat(temp, 1.1);
  pnt_solver.SeedToughSatGroup(1.1);

  for (unsigned int prn = 0; prn < 10; ++prn) {
    temp.sat_prn = prn;
    for (unsigned int j = 0; j < 7; ++j) {
      pnt_solver.IsToughNewSat(temp);
    }
  }

  pnt_solver.tough_fixing_pool_.clear();

  // 17) SynchronizeBaser
  unsigned int gnss_week_num = 0;
  double gnss_week_second_s = 0;
  apollo::localization::local_gnss::EpochObservation baser_obs;
  pnt_solver.baser_obs_.resize(0);
  ASSERT_FALSE(pnt_solver.SynchronizeBaser(gnss_week_num, gnss_week_second_s,
                                           &baser_obs));

  baser_obs.set_gnss_week(gnss_week_num);
  for (unsigned int i = 0; i < 10; ++i) {
    baser_obs.set_gnss_second_s(i * 3);
    pnt_solver.baser_obs_.push_back(baser_obs);
  }
  // max_second = 9 * 3 = 27.0
  ASSERT_TRUE(pnt_solver.SynchronizeBaser(gnss_week_num, 28.0, &baser_obs));
  ASSERT_TRUE(pnt_solver.SynchronizeBaser(gnss_week_num, 26.9, &baser_obs));
  ASSERT_TRUE(pnt_solver.SynchronizeBaser(gnss_week_num, 16.9, &baser_obs));
  ASSERT_TRUE(pnt_solver.SynchronizeBaser(gnss_week_num, 14.9, &baser_obs));

  // 18) CheckSlip
  pnt_solver.rover_preprocessor_.Reset();
  pnt_solver.baser_preprocessor_.Reset();

  ASSERT_TRUE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 1, 2, 2));

  temp.band_id = apollo::drivers::gnss::GPS_L1;
  temp.sat_prn = 1;
  pnt_solver.rover_preprocessor_.slip_recorder_.insert(
      std::map<SlipKey, double>::value_type(temp, 0.9));
  pnt_solver.baser_preprocessor_.slip_recorder_.insert(
      std::map<SlipKey, double>::value_type(temp, 0.9));

  temp.sat_prn = 2;
  pnt_solver.rover_preprocessor_.slip_recorder_.insert(
      std::map<SlipKey, double>::value_type(temp, 0.8));
  temp.sat_prn = 3;
  pnt_solver.baser_preprocessor_.slip_recorder_.insert(
      std::map<SlipKey, double>::value_type(temp, 0.7));

  pnt_solver.enable_cycle_slip_fix_ = false;
  ASSERT_FALSE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 1, 2, 2));
  ASSERT_FALSE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 2, 2, 2));
  ASSERT_FALSE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 3, 2, 2));

  pnt_solver.enable_cycle_slip_fix_ = true;
  ASSERT_FALSE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 1, 2, 2));
  ASSERT_FALSE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 1, 2, 0));
  ASSERT_FALSE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 1, 0, 2));
  ASSERT_FALSE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 1, 0, 0));
  ASSERT_FALSE(pnt_solver.CheckSlip(apollo::drivers::gnss::GPS_L1, 3, 0, 0));

  // 19 SaveGnssEphemris
  GnssEphemeris gnss_orbit;
  gnss_orbit.set_gnss_type(apollo::drivers::gnss::SYS_UNKNOWN);
  ASSERT_FALSE(pnt_solver.SaveGnssEphemris(gnss_orbit));

  // 20)GetStdThresForNewSat
  double std_v = 0.01;
  pnt_solver.GetStdThresForNewSat(2.0, std_v);
  pnt_solver.GetStdThresForNewSat(4.0, std_v);
  pnt_solver.GetStdThresForNewSat(6.0, std_v);
  pnt_solver.GetStdThresForNewSat(10.0, std_v);
  pnt_solver.GetStdThresForNewSat(20.0, std_v);
  std_v = 0.031;
  pnt_solver.GetStdThresForNewSat(20.0, std_v);

  // 21)WeightOnPostResidual
  unsigned int size = 8;
  Eigen::MatrixXd px(size, size);
  px.setIdentity();

  std::vector<unsigned int> range_type;
  range_type.resize(size);
  Eigen::MatrixXd post_res(size, 1);
  post_res.setZero();

  for (unsigned int i = 0; i < 4; ++i) {
    range_type[i] = 0;
    range_type[i + 4] = 1;
  }
  post_res(0, 0) = 21 * std_v;
  post_res(1, 0) = 16 * std_v;
  post_res(2, 0) = 11 * std_v;
  post_res(3, 0) = 9 * std_v;
  post_res(4, 0) = 5 * std_v;
  post_res(5, 0) = 3.5 * std_v;
  post_res(6, 0) = 2.5 * std_v;
  post_res(7, 0) = 1.5 * std_v;
  pnt_solver.WeightOnPostResidual(std_v, range_type, post_res, &px);

  // 22)BoundRangePrecision
  pnt_solver.range_precision_ = 2 * 1.0 * 1.0;
  pnt_solver.BoundRangePrecision(3.0);
  pnt_solver.BoundRangePrecision(1.0);

  // 24) CheckBaserObs
  ASSERT_FALSE(pnt_solver.CheckBaserObs(1.0, 1000));
  ASSERT_FALSE(pnt_solver.CheckBaserObs(100.0, 1.0));
  ASSERT_FALSE(pnt_solver.CheckBaserObs(-100.0, 1.0));
  ASSERT_TRUE(pnt_solver.CheckBaserObs(1.0, 1.0));

  // 25) valid ambiguity resolution result
  ASSERT_FALSE(pnt_solver.CheckLambda(-0.1, 0.1));
  ASSERT_FALSE(pnt_solver.CheckLambda(5.0, 100.0));
  ASSERT_TRUE(pnt_solver.CheckLambda(5.0, 0.1));

  // 26) valid IsAbnormalFixedSolutionWithNewPhase
  ASSERT_FALSE(pnt_solver.IsAbnormalFixedSolutionWithNewPhase(false, 0.1, 0));
  ASSERT_FALSE(pnt_solver.IsAbnormalFixedSolutionWithNewPhase(false, 0.001, 1));
  ASSERT_FALSE(pnt_solver.IsAbnormalFixedSolutionWithNewPhase(true, 0.001, 1));
  ASSERT_TRUE(pnt_solver.IsAbnormalFixedSolutionWithNewPhase(true, 0.5, 1));

  // 27) IsNeedRecursion
  ASSERT_FALSE(pnt_solver.IsNeedRecursion(false, 0.1, 0, 10));
  ASSERT_TRUE(pnt_solver.IsNeedRecursion(false, 0.001, 5, 4));
  ASSERT_TRUE(pnt_solver.IsNeedRecursion(true, 0.5, 1, 10));

  // 28) update ratio
  ASSERT_EQ(3.0, pnt_solver.BoundRatio(3.0));
  ASSERT_EQ(1000.0, pnt_solver.BoundRatio(10000.0));

  // 29) valid delat pos caused by new satellites, CheckNewsatSolution
  PointThreeDim denu(0.01, 0.01, 0.03);
  const double thres = 0.1;
  ASSERT_TRUE(pnt_solver.CheckNewsatSolution(denu, thres));

  ASSERT_TRUE(pnt_solver.CheckNewsatSolution(denu, thres, 0.01));
  ASSERT_FALSE(pnt_solver.CheckNewsatSolution(denu, thres, 0.10));

  denu.z = 0.3;
  ASSERT_FALSE(pnt_solver.CheckNewsatSolution(denu, thres));

  denu.z = 0.03;
  denu.y = 0.3;
  ASSERT_FALSE(pnt_solver.CheckNewsatSolution(denu, thres));

  denu.z = 0.03;
  denu.y = 0.03;
  denu.x = 0.3;
  ASSERT_FALSE(pnt_solver.CheckNewsatSolution(denu, thres));

  // 29)GetInvalidRtkDetail
  pnt_solver.invalid_rtk_sub_infor_ = "ut_test";
  pnt_solver.GetInvalidRtkDetail();

  // 30) IsEnabledCodeDiff
  ASSERT_FALSE(pnt_solver.IsEnabledCodeDiff(61.0, 10.0));
  ASSERT_FALSE(pnt_solver.IsEnabledCodeDiff(1.0, 51000.0));
  ASSERT_TRUE(pnt_solver.IsEnabledCodeDiff(1.0, 1000.0));

  // 31) IsEnabledPhaseDiff
  ASSERT_FALSE(pnt_solver.IsEnabledPhaseDiff(16.0, 10.0));
  ASSERT_FALSE(pnt_solver.IsEnabledPhaseDiff(1.0, 26000.0));
  ASSERT_TRUE(pnt_solver.IsEnabledPhaseDiff(1.0, 1000.0));

  // 32)UpdateBaseCoor
  EpochObservation rover_obs;
  baser_obs.Clear();
  rover_obs.Clear();
  ASSERT_EQ(0, pnt_solver.UpdateBaseCoor(rover_obs, baser_obs));

  pnt_solver.baser_coor_.x = 0.0;
  pnt_solver.baser_coor_.y = 0.0;
  pnt_solver.baser_coor_.z = 0.0;
  baser_obs.set_position_x(0.0);
  baser_obs.set_position_y(0.0);
  baser_obs.set_position_z(0.0);
  rover_obs.set_gnss_week(0.0);
  rover_obs.set_gnss_second_s(0.0);
  baser_obs.set_gnss_week(0.0);
  baser_obs.set_gnss_second_s(0.0);
  ASSERT_EQ(1, pnt_solver.UpdateBaseCoor(rover_obs, baser_obs));

  baser_obs.set_position_x(1.0);
  baser_obs.set_position_y(1.0);
  baser_obs.set_position_z(1.0);
  ASSERT_EQ(1, pnt_solver.UpdateBaseCoor(rover_obs, baser_obs));

  // 33) CheckSatObsForSPP
  one_sat.Clear();
  one_sat.set_band_obs_num(0);
  ASSERT_FALSE(pnt_solver.CheckSatObsForSPP(one_sat));

  band_obs.Clear();
  band_obs.set_pseudo_range(0.0);
  *one_sat.add_band_obs() = band_obs;
  one_sat.set_band_obs_num(one_sat.band_obs_size());
  ASSERT_FALSE(pnt_solver.CheckSatObsForSPP(one_sat));

  const double c_light_speed = 2.99792458e8;
  const double min_time_signal = 0.030;
  const double max_time_signal = 0.200;

  // 20804163.233 = 69 ms
  // 36804163.233 = 122.7 ms
  band_obs.set_pseudo_range((min_time_signal - 0.01) * c_light_speed);
  *one_sat.mutable_band_obs(0) = band_obs;
  ASSERT_FALSE(pnt_solver.CheckSatObsForSPP(one_sat));

  band_obs.set_pseudo_range((max_time_signal + 0.01) * c_light_speed);
  *one_sat.mutable_band_obs(0) = band_obs;
  ASSERT_FALSE(pnt_solver.CheckSatObsForSPP(one_sat));

  band_obs.set_pseudo_range(20804163.233);
  *one_sat.mutable_band_obs(0) = band_obs;
  ASSERT_TRUE(pnt_solver.CheckSatObsForSPP(one_sat));

  // 34) IsSafeFixedSolution
  unsigned int gnss_type_size = 3;
  pnt_solver.b_rtkfixed_ = false;
  pnt_solver.amb_tracker_.amb_fixed_.clear();
  unsigned int NumFixedPhase = pnt_solver.amb_tracker_.GetNumFixedAmb();
  ASSERT_FALSE(pnt_solver.IsSafeFixedSolution(gnss_type_size, NumFixedPhase));

  pnt_solver.b_rtkfixed_ = true;
  ASSERT_FALSE(pnt_solver.IsSafeFixedSolution(gnss_type_size, NumFixedPhase));
  pnt_solver.b_rtkfixed_ = true;
  ASSERT_FALSE(
      pnt_solver.IsSafeFixedSolution(gnss_type_size, gnss_type_size * 5 - 1));

  gnss_type_size = 2;
  for (unsigned int prn = 1; prn < 20; ++prn) {
    pnt_solver.amb_tracker_.SetFixedAmb(apollo::drivers::gnss::GPS_L1, prn,
                                        1.0);
  }

  pnt_solver.b_rtkfixed_ = true;
  NumFixedPhase = pnt_solver.amb_tracker_.GetNumFixedAmb();
  pnt_solver.rtk_dop_phase_.x = 15.0;
  pnt_solver.rtk_dop_phase_.y = 15.0;
  pnt_solver.rtk_dop_phase_.z = 15.0;
  ASSERT_FALSE(
      pnt_solver.IsSafeFixedSolution(gnss_type_size, gnss_type_size * 5 + 1));

  pnt_solver.b_rtkfixed_ = true;
  pnt_solver.rtk_dop_phase_.x = 1.0;
  pnt_solver.rtk_dop_phase_.y = 1.0;
  pnt_solver.rtk_dop_phase_.z = 1.0;
  ASSERT_TRUE(pnt_solver.IsSafeFixedSolution(gnss_type_size, NumFixedPhase));

  pnt_solver.rtk_dop_phase_.z = 10.0;
  ASSERT_TRUE(pnt_solver.IsSafeFixedSolution(gnss_type_size, NumFixedPhase));

  pnt_solver.rtk_dop_phase_.y = 10.0;
  pnt_solver.rtk_dop_phase_.z = 10.0;
  for (unsigned int i = 0; i < 10; ++i) {
    pnt_solver.IsSafeFixedSolution(gnss_type_size, NumFixedPhase);
  }

  pnt_solver.std_rtk_.x = 1.0;
  pnt_solver.std_rtk_.y = 1.0;
  pnt_solver.std_rtk_.z = 1.0;
  pnt_solver.std_rtk_thres_of_phase_dop_.x = 1.0;
  pnt_solver.std_rtk_thres_of_phase_dop_.y = 1.0;
  pnt_solver.std_rtk_thres_of_phase_dop_.z = 1.0;
  pnt_solver.BoundedRtkStd();

  pnt_solver.std_rtk_thres_of_phase_dop_.z = 100.0;
  pnt_solver.BoundedRtkStd();

  std::vector<ObsKey> phase_recorder;
  phase_recorder.resize(0);
  std_v = 0.02;
  range_type.resize(0);

  const unsigned int size_post_res = 10;
  post_res.resize(size_post_res, 1);
  post_res(0, 0) = 21 * std_v;
  post_res(1, 0) = 16 * std_v;
  post_res(2, 0) = 11 * std_v;
  post_res(3, 0) = 9 * std_v;
  post_res(4, 0) = 5 * std_v;
  post_res(5, 0) = 3.5 * std_v;
  post_res(6, 0) = 2.5 * std_v;
  post_res(7, 0) = 1.5 * std_v;

  double band_res_sum[apollo::drivers::gnss::GLO_G3] = {0.0};
  pnt_solver.RemoveAmbWithAbnormalRes(phase_recorder, std_v, range_type,
                                      post_res, band_res_sum);

  for (unsigned int i = 0; i < size_post_res; ++i) {
    if (i % 2 == 0) {
      range_type.push_back(0);
    } else {
      range_type.push_back(1);
      phase_recorder.push_back(ObsKey(apollo::drivers::gnss::GPS_L1, i + 1));
    }
  }

  pnt_solver.RemoveAmbWithAbnormalRes(phase_recorder, std_v, range_type,
                                      post_res, band_res_sum);
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
