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

#include "modules/localization/msf/local_gnss/pvt_gnss.h"

namespace apollo {
namespace localization {
namespace local_gnss {

bool KepplerPvt::GetClock(const double observe_time,
                           const KepplerOrbit& current_eph, double* clk_bias,
                           double* clk_drift) {
  // declare a wgs84
  double gm = GetEarthGm();
  double gme64_sqrt = sqrt(gm);

  double n0 = gme64_sqrt /
              (current_eph.roota() * current_eph.roota() * current_eph.roota());
  double n = n0 + current_eph.deltan();
  // reference to IODC
  double toc = current_eph.toc() +
               current_eph.week_num() *
               apollo::localization::local_gnss::SECOND_PER_WEEK;
  double tk = observe_time - toc;
  CheckValidReferIOD(&tk);

  double mk = current_eph.m0() + n * tk;
  // computer ek
  double ek = ComputeEhpEk(mk, current_eph.e(), 10);

  double sinek = sin(ek);
  // clock bias with relativistic corrected
  *clk_bias = current_eph.af0() + current_eph.af1() * tk +
             current_eph.af2() * tk * tk -
             4.443e-10 * current_eph.e() * current_eph.roota() * sinek;
  *clk_drift = current_eph.af1() + current_eph.af2() * tk;
  return true;
}

bool KepplerPvt::GetPositionVelocity(const double observe_time,
                                       const KepplerOrbit& current_eph,
                                       PointThreeDim* position,
                                       PointThreeDim* velocity) {
  double gm = GetEarthGm();
  double gme_sqrt = sqrt(gm);
  double earth_rate = GetEarthRotation();

  // compute SV mean rate of angle
  double n0 = gme_sqrt /
              (current_eph.roota() * current_eph.roota() * current_eph.roota());
  double n = n0 + current_eph.deltan();
  // compute time tk,reference to IODE
  double toe = current_eph.toe() +
               current_eph.week_num() *
               apollo::localization::local_gnss::SECOND_PER_WEEK;
  double tk = observe_time - toe;
  CheckValidReferIOD(&tk);

  double mk = current_eph.m0() + n * tk;
  // computer ek
  double ek = ComputeEhpEk(mk, current_eph.e(), 10);

  // computer vk
  double vk = atan2(sqrt(1.0 - current_eph.e() * current_eph.e()) * sin(ek),
                    cos(ek) - current_eph.e());
  // compute phik
  double phik = vk + current_eph.omega();
  double c2u = cos(2.0 * phik);
  double s2u = sin(2.0 * phik);
  // compute deltau,deltar,deltai
  double deltau = current_eph.cuc() * c2u + current_eph.cus() * s2u;
  double deltar = current_eph.crc() * c2u + current_eph.crs() * s2u;
  double deltai = current_eph.cic() * c2u + current_eph.cis() * s2u;

  // computer uk, rk and ik
  double uk = phik + deltau;
  double rk = current_eph.roota() * current_eph.roota() *
                  (1.0 - current_eph.e() * cos(ek)) +
              deltar;
  double ik = current_eph.i0() + deltai + current_eph.idot() * tk;

  // compute xk,yk
  double xk = rk * cos(uk);
  double yk = rk * sin(uk);

  // when BDS is GEO, update omegak and wdot
  unsigned int prn = current_eph.sat_prn();

  // compute omegak
  double omegak = current_eph.omega0() +
                  (current_eph.omegadot() - earth_rate) * tk -
                  earth_rate * current_eph.toe();
  double wdot = current_eph.omegadot() - earth_rate;

  if (IsGEO(prn)) {
    omegak = current_eph.omega0() + (current_eph.omegadot()) * tk -
             earth_rate * current_eph.toe();
    wdot = current_eph.omegadot();
  }
  double sw = sin(omegak);
  double cw = cos(omegak);
  double si = sin(ik);
  double ci = cos(ik);

  // compute ECEF
  position->x = xk * cw - yk * ci * sw;
  position->y = xk * sw + yk * ci * cw;
  position->z = yk * si;

  double esdot = n / (1 - current_eph.e() * cos(ek));
  double u0dot = sqrt(1 - current_eph.e() * current_eph.e()) * esdot /
                 (1 - current_eph.e() * cos(ek));

  double idot = current_eph.idot() +
                2 * (current_eph.cis() * c2u - current_eph.cic() * s2u) * u0dot;
  double rdot = current_eph.roota() * current_eph.roota() * current_eph.e() *
                    sin(ek) * esdot +
                2 * (current_eph.crs() * c2u - current_eph.crc() * s2u) * u0dot;
  double udot =
      u0dot + 2 * (current_eph.cus() * c2u - current_eph.cuc() * s2u) * u0dot;
  double x0dot = cos(uk) * rdot - rk * sin(uk) * udot;
  double y0dot = sin(uk) * rdot + rk * cos(uk) * udot;
  velocity->x = cw * x0dot - sw * ci * y0dot - (xk * sw + yk * cw * ci) * wdot +
               yk * sw * si * idot;
  velocity->y = sw * x0dot + cw * ci * y0dot + (xk * cw - yk * sw * ci) * wdot +
               yk * cw * si * idot;
  velocity->z = si * y0dot + yk * ci * idot;
  if (IsGEO(prn)) {
    double x64 = position->x;
    double y64 = position->y;
    double z64 = position->z;
    double angle = -5.0 * PI / 180.0;
    double sina = sin(angle);
    double cosa = cos(angle);
    double geo_x = position->x;
    double geo_y = cosa * position->y + sina * position->z;
    double geo_z = cosa * position->z - sina * position->y;
    double omega_off = earth_rate * tk;
    double sin_omega = sin(omega_off);
    double cos_omega = cos(omega_off);
    position->x = geo_x * cos_omega + geo_y * sin_omega;
    position->y = -geo_x * sin_omega + geo_y * cos_omega;
    position->z = geo_z;
    // velocity
    double v_x164 = cos_omega * velocity->x + sin_omega * cosa * velocity->y +
                    sin_omega * sina * velocity->z;
    double v_y164 = (-sin_omega) * velocity->x +
                    cos_omega * cosa * velocity->y +
                    cos_omega * sina * velocity->z;
    double v_z164 = (-sina) * velocity->y + cosa * velocity->z;

    double v_x264 =
        ((-sin_omega) * x64 + cos_omega * cosa * y64 + cos_omega * sina * z64) *
        earth_rate;
    double v_y264 = ((-cos_omega) * x64 + (-sin_omega) * cosa * y64 +
                     (-sin_omega) * sina * z64) *
                    earth_rate;
    double v_z264 = 0;
    velocity->x = v_x164 + v_x264;
    velocity->y = v_y164 + v_y264;
    velocity->z = v_z164 + v_z264;
  }
  return true;
}

bool GlonassPvt::GetClock(const double observe_time,
                           const GlonassOrbit& current_eph, double* clk_bias,
                           double* clk_drift) {
  double toc =
      SECOND_PER_WEEK * current_eph.week_num() + current_eph.week_second_s();
  double dt = observe_time - toc;
  *clk_bias = current_eph.clock_offset() + current_eph.clock_drift() * dt;
  *clk_drift = current_eph.clock_drift();
  return true;
}

bool GlonassPvt::GetPositionVelocity(const double observe_time,
                                       const GlonassOrbit& current_eph,
                                       PointThreeDim* position,
                                       PointThreeDim* velocity) {
  double toe =
      SECOND_PER_WEEK * current_eph.week_num() + current_eph.week_second_s();
  double dt = observe_time - toe;
  double sign = 0;
  if (dt >= 0) {
    sign = 1;
  } else {
    sign = -1;
  }
  // default integration step
  const double default_step = 60.0;
  double r0[3] = {current_eph.position_x(), current_eph.position_y(),
                  current_eph.position_z()};
  double v0[3] = {current_eph.velocity_x(), current_eph.velocity_y(),
                  current_eph.velocity_z()};
  double a0[3] = {current_eph.accelerate_x(), current_eph.accelerate_y(),
                  current_eph.accelerate_z()};
  double r[3] = {0.0};
  double v[3] = {0.0};
  double y[6][6] = {0.0};
  do {
    double step = 0.0;
    if (fabs(dt) > default_step) {
      step = default_step * sign;
    } else {
      step = dt;
    }
    // compute right function at the first time
    for (int i = 0; i < 3; i++) {
      r[i] = r0[i];
      v[i] = v0[i];
    }
    compute_right_function(r, v, a0, y[0]);
    // compute right function at the second time
    for (int i = 0; i < 3; i++) {
      v[i] = v0[i] + y[0][3 + i] * step / 2.0;
      r[i] = r0[i] + y[0][i] * step / 2.0;
    }
    compute_right_function(r, v, a0, y[1]);
    // compute right function at the third time
    for (int i = 0; i < 3; i++) {
      r[i] = r0[i] + y[1][i] * step / 2.0;
      v[i] = v0[i] + y[1][3 + i] * step / 2.0;
    }
    compute_right_function(r, v, a0, y[2]);
    // compute right function at the fourth time
    for (int i = 0; i < 3; i++) {
      r[i] = r0[i] + y[2][i] * step;
      v[i] = v0[i] + y[2][3 + i] * step;
    }
    compute_right_function(r, v, a0, y[3]);
    // integration result
    for (int i = 0; i < 3; i++) {
      r0[i] = r0[i] +
              (y[0][i] + y[1][i] * 2.0 + y[2][i] * 2.0 + y[3][i]) * step / 6.0;
      v0[i] = v0[i] + (y[0][i + 3] + y[1][i + 3] * 2.0 + y[2][i + 3] * 2.0 +
                       y[3][i + 3]) *
                          step / 6.0;
    }
    // update toe time
    toe += step;
    dt = observe_time - toe;
  } while (fabs(dt) > 1E-8);

  position->x = r0[0];
  position->y = r0[1];
  position->z = r0[2];
  velocity->x = v0[0];
  velocity->y = v0[1];
  velocity->z = v0[2];
  return true;
}

bool GlonassPvt::compute_right_function(double pos[3], double vel[3],
                                        double acc[3], double vel_acc_next[6]) {
  double distance = 0.0;
  for (int i = 0; i < 3; ++i) {
    distance += pos[i] * pos[i];
  }
  distance = sqrt(distance);
  double gm_n = GetEarthGm() / (distance * distance);
  double r_n = GetEarthSemiMajor() / distance;
  double pos_n[3] = {0.0};
  for (int i = 0; i < 3; ++i) {
    vel_acc_next[i] = vel[i];
    pos_n[i] = pos[i] / distance;
  }
  double geo_po_j02 = -GetEarthJ20();
  double earth_rate_pz90 = GetEarthRotation();

  double tmp = 1.5 * geo_po_j02 * gm_n * pow(r_n, 2.0) *
               (1.0 - 5.0 * pow(pos_n[2], 2.0));

  vel_acc_next[3] = -gm_n * pos_n[0] - tmp * pos_n[0] +
                    pow(earth_rate_pz90, 2.0) * pos[0] +
                    2.0 * earth_rate_pz90 * vel[1] + acc[0];

  vel_acc_next[4] = -gm_n * pos_n[1] - tmp * pos_n[1] +
                    pow(earth_rate_pz90, 2.0) * pos[1] -
                    2.0 * earth_rate_pz90 * vel[0] + acc[1];

  vel_acc_next[5] = -gm_n * pos_n[2] -
                    1.5 * geo_po_j02 * gm_n * pow(r_n, 2.0) *
                        (3.0 - 5.0 * pow(pos_n[2], 2.0)) * pos_n[2] +
                    acc[2];
  return true;
}

bool GlonassPvt::position_pz90_to_wgs84(double pz90[3], double wgs84[3]) {
  double dx0[3] = {0.0, 2.5, 0.0};
  double r[3][3] = {1.0, -1.9E-6, 0.0, 1.9E-6, 1.0, 0.0, 0.0, 0.0, 1.0};
  for (int i = 0; i < 3; ++i) {
    wgs84[i] = 0;
    for (int j = 0; j < 3; ++j) {
      wgs84[i] += r[i][j] * pz90[j];
    }
    wgs84[i] += dx0[i];
  }
  return true;
}

bool GlonassPvt::velocity_pz90_to_wgs84(double pz90[3], double wgs84[3]) {
  double r[3][3] = {1.0, -1.9E-6, 0.0, 1.9E-6, 1.0, 0.0, 0.0, 0.0, 1.0};
  for (int i = 0; i < 3; ++i) {
    wgs84[i] = 0;
    for (int j = 0; j < 3; ++j) {
      wgs84[i] += r[i][j] * pz90[j];
    }
  }
  return true;
}

bool GlonassPvt::TransferPz90ToWgs84(PointThreeDim* position,
                               PointThreeDim* velocity) {
  double pos_pz90[3] = {position->x, position->y, position->z};
  double pos_wgs84[3] = {0.0};
  position_pz90_to_wgs84(pos_pz90, pos_wgs84);
  position->x = pos_wgs84[0];
  position->y = pos_wgs84[1];
  position->z = pos_wgs84[2];

  double vel_pz90[3] = {velocity->x, velocity->y, velocity->z};
  double vel_wgs84[3] = {0.0};
  velocity_pz90_to_wgs84(vel_pz90, vel_wgs84);
  velocity->x = vel_wgs84[0];
  velocity->y = vel_wgs84[1];
  velocity->z = vel_wgs84[2];
  return true;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
