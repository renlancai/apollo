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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_CONSTANTS_HPP_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_GNSS_CONSTANTS_HPP_
namespace apollo {
namespace localization {
namespace local_gnss {

// PI
const double PI = 3.141592653589793238462643383280;
const double TWO_PI = 6.283185307179586476925286766559;
const double SEMI_PI = 1.772453850905516027298167483341;

// m/s, speed of light
const double C_MPS = 2.99792458e8;

// time
const int SECOND_PER_WEEK = 604800;

class GnssEllipsoidParam {
 public:
  inline GnssEllipsoidParam() {}
  inline ~GnssEllipsoidParam() {}

  GnssEllipsoidParam& operator=(GnssEllipsoidParam& other) { // NOLINT
    earth_semi_major_ = other.GetEarthSemiMajor();
    earth_eccentricity_ = other.GetEarthEccentricity();
    earth_gm_ = other.GetEarthGm();
    earth_sell_rotation_ = other.GetEarthRotation();
    light_speed_ = other.GetLightSpeed();
    return *this;
  }

  inline double GetEarthSemiMajor() { return earth_semi_major_; }
  inline double GetEarthEccentricity() { return earth_eccentricity_; }
  inline double GetEarthGm() { return earth_gm_; }
  inline double GetEarthRotation() { return earth_sell_rotation_; }
  inline double GetLightSpeed() { return light_speed_; }
  inline double GetEarthJ20() { return _earth_j20; }

  inline double GetRelativityConst() {
    // relativity constant (sec/sqrt(m))
    const double rel_const = -4.442807633e-10;
    return rel_const;
  }

  inline void SetEarthParams(const double& earth_semi_major,
                               const double& earth_eccentricity,
                               const double& earth_gm,
                               const double& earth_sell_rotation,
                               const double& light_speed = 299792458.0) {
    SetEarthSemiMajor(earth_semi_major);
    SetEarthEccentricity(earth_eccentricity);
    SetEarthGm(earth_gm);
    SetEarthRotation(earth_sell_rotation);
    SetLightSpeed(light_speed);
  }

  inline void SetEarthSemiMajor(const double& earth_semi_major) {
    earth_semi_major_ = earth_semi_major;
  }
  inline void SetEarthEccentricity(const double& earth_eccentricity) {
    earth_eccentricity_ = earth_eccentricity;
  }
  inline void SetEarthGm(const double& earth_gm) { earth_gm_ = earth_gm; }
  inline void SetEarthRotation(const double& earth_sell_rotation) {
    earth_sell_rotation_ = earth_sell_rotation;
  }
  inline void SetLightSpeed(const double& light_speed) {
    light_speed_ = light_speed;
  }
  inline void SetEarthJ20(const double& earth_j20) { _earth_j20 = earth_j20; }

 private:
  double earth_semi_major_ = 6378137;
  double earth_eccentricity_ = 8.1819190842622e-2;
  double earth_gm_ = 3986004.418e8;
  double earth_sell_rotation_ = 7.292115e-5;
  double light_speed_ = 299792458.0;
  double _earth_j20 = 0 - 1.08262575e-3;
};

class GpsEllipsoidParam : public GnssEllipsoidParam {
 public:
  inline GpsEllipsoidParam() {
    // refer to GPS-ICD ~~
    SetEarthSemiMajor(6378137.0);
    SetEarthEccentricity(8.1819190842622e-2);
    SetEarthGm(3.986005e14);
    SetEarthRotation(7.2921151467e-5);
    SetLightSpeed(2.99792458e8);
  }
};

class BdsEllipsoidParam : public GnssEllipsoidParam {
 public:
  inline BdsEllipsoidParam() {
    // refer to BDS-ICD ~~
    SetEarthSemiMajor(6378137.0);
    // flattening : f = 1 / 298.257222101
    SetEarthEccentricity(8.1819190842622e-2);
    SetEarthGm(3.986004418e14);
    SetEarthRotation(7.2921150e-5);
    SetLightSpeed(2.99792458e8);
  }
};

class GloEllipsoidParam : public GnssEllipsoidParam {
 public:
  inline GloEllipsoidParam() {
    // refer to Glo-ICD Table 4.1
    SetEarthSemiMajor(6378136.0);
    // flattening : f = 1 / 298.25784
    SetEarthEccentricity(8.1819106432923e-2);
    SetEarthGm(398600.4418e9);
    SetEarthRotation(7.292115e-5);
    SetLightSpeed(299792458);
    SetEarthJ20(-1.08262575e-3);
  }
};

class GalEllipsoidParam : public GnssEllipsoidParam {
 public:
  inline GalEllipsoidParam() {
    // refer to GAL-ICD ~~
    SetEarthSemiMajor(6378137.0);
    SetEarthEccentricity(8.1819106432923e-2);
    SetEarthGm(3.986004418e14);
    SetEarthRotation(7.2921151467E-5);
    SetLightSpeed(299792458);
  }
};

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
#endif
