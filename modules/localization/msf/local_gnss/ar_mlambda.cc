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

#include "modules/localization/msf/local_gnss/ar_mlambda.h"

namespace apollo {
namespace localization {
namespace local_gnss {
/* a temporal ratio threshold look up table for fixed failure rate, refer to:
   Sandra Verhagen ,The ratio test for future GNSS ambiguity resolution, 2009.
*/
const unsigned int max_number_index = 30;
const unsigned int max_number_dd_amb = 16;
const double ILS_failure_index[max_number_index] = {
    0.0010, 0.0012, 0.0015, 0.0020, 0.0050, 0.0100, 0.0150, 0.0200,
    0.0250, 0.0300, 0.0350, 0.0400, 0.0450, 0.0500, 0.0550, 0.0600,
    0.0650, 0.0700, 0.0750, 0.0800, 0.0850, 0.0900, 0.0950, 0.1000,
    0.1200, 0.1500, 0.2000, 0.2500, 0.2600, 1.0000};
const double ratio_look_up_table[max_number_index][max_number_dd_amb] = {
    {1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
     1.00, 1.00, 1.00, 1.00},  // 0.0010
    {0.94, 0.94, 0.94, 0.94, 0.94, 0.94, 0.94, 0.94, 0.94, 0.96, 0.96, 0.96,
     0.96, 0.96, 0.96, 0.96},  // 0.0012
    {0.87, 0.87, 0.87, 0.88, 0.88, 0.89, 0.89, 0.89, 0.89, 0.91, 0.92, 0.92,
     0.92, 0.92, 0.93, 0.93},  // 0.0015
    {0.78, 0.78, 0.78, 0.80, 0.80, 0.81, 0.82, 0.83, 0.84, 0.84, 0.86, 0.86,
     0.86, 0.87, 0.88, 0.88},  // 0.0020
    {0.54, 0.54, 0.54, 0.57, 0.57, 0.59, 0.64, 0.64, 0.68, 0.68, 0.69, 0.71,
     0.72, 0.73, 0.74, 0.75},  // 0.0050
    {0.36, 0.36, 0.38, 0.41, 0.43, 0.45, 0.51, 0.52, 0.57, 0.57, 0.59, 0.61,
     0.62, 0.64, 0.65, 0.67},  // 0.0100
    {0.27, 0.27, 0.29, 0.32, 0.36, 0.38, 0.43, 0.46, 0.51, 0.52, 0.53, 0.55,
     0.56, 0.58, 0.60, 0.62},  // 0.0150
    {0.21, 0.21, 0.24, 0.27, 0.30, 0.33, 0.38, 0.42, 0.46, 0.48, 0.49, 0.52,
     0.52, 0.55, 0.57, 0.59},  // 0.0200
    {0.19, 0.19, 0.22, 0.24, 0.27, 0.29, 0.35, 0.39, 0.42, 0.45, 0.46, 0.49,
     0.49, 0.53, 0.54, 0.56},  // 0.0250
    {0.17, 0.17, 0.20, 0.22, 0.24, 0.26, 0.32, 0.35, 0.40, 0.42, 0.44, 0.47,
     0.47, 0.51, 0.53, 0.54},  // 0.0300
    {0.16, 0.16, 0.18, 0.20, 0.22, 0.24, 0.30, 0.33, 0.38, 0.40, 0.42, 0.45,
     0.45, 0.49, 0.51, 0.53},  // 0.0350
    {0.15, 0.15, 0.17, 0.18, 0.21, 0.22, 0.27, 0.31, 0.36, 0.39, 0.40, 0.44,
     0.44, 0.48, 0.50, 0.52},  // 0.0400
    {0.14, 0.14, 0.16, 0.17, 0.19, 0.21, 0.26, 0.29, 0.34, 0.38, 0.39, 0.43,
     0.44, 0.48, 0.49, 0.51},  // 0.0450
    {0.13, 0.13, 0.15, 0.16, 0.18, 0.19, 0.24, 0.28, 0.32, 0.37, 0.39, 0.42,
     0.43, 0.47, 0.48, 0.51},  // 0.0500
    {0.12, 0.12, 0.14, 0.15, 0.17, 0.18, 0.23, 0.27, 0.31, 0.36, 0.38, 0.41,
     0.42, 0.46, 0.48, 0.50},  // 0.0550
    {0.12, 0.12, 0.13, 0.15, 0.16, 0.17, 0.21, 0.26, 0.30, 0.36, 0.37, 0.40,
     0.42, 0.46, 0.47, 0.50},  // 0.0600
    {0.11, 0.11, 0.12, 0.14, 0.15, 0.16, 0.20, 0.26, 0.30, 0.35, 0.36, 0.40,
     0.41, 0.45, 0.46, 0.50},  // 0.0650
    {0.11, 0.11, 0.12, 0.13, 0.14, 0.15, 0.20, 0.25, 0.30, 0.35, 0.36, 0.39,
     0.41, 0.45, 0.46, 0.49},  // 0.0700
    {0.10, 0.10, 0.11, 0.12, 0.13, 0.15, 0.19, 0.24, 0.29, 0.34, 0.36, 0.39,
     0.41, 0.44, 0.45, 0.49},  // 0.0750
    {0.10, 0.10, 0.10, 0.12, 0.13, 0.14, 0.19, 0.23, 0.29, 0.34, 0.35, 0.38,
     0.40, 0.44, 0.45, 0.49},  // 0.0800
    {0.10, 0.10, 0.10, 0.11, 0.12, 0.14, 0.18, 0.23, 0.28, 0.33, 0.35, 0.38,
     0.40, 0.44, 0.45, 0.49},  // 0.0850
    {0.09, 0.09, 0.09, 0.11, 0.12, 0.13, 0.18, 0.22, 0.28, 0.33, 0.35, 0.38,
     0.40, 0.43, 0.45, 0.49},  // 0.0900
    {0.09, 0.09, 0.09, 0.10, 0.11, 0.13, 0.18, 0.22, 0.28, 0.33, 0.34, 0.38,
     0.40, 0.43, 0.45, 0.48},  // 0.0950
    {0.09, 0.09, 0.09, 0.10, 0.11, 0.12, 0.17, 0.22, 0.27, 0.32, 0.34, 0.37,
     0.40, 0.43, 0.44, 0.48},  // 0.1000
    {0.08, 0.08, 0.08, 0.09, 0.09, 0.11, 0.16, 0.21, 0.26, 0.31, 0.33, 0.37,
     0.39, 0.42, 0.44, 0.48},  // 0.1200
    {0.07, 0.07, 0.07, 0.08, 0.08, 0.10, 0.16, 0.20, 0.25, 0.29, 0.32, 0.35,
     0.39, 0.41, 0.44, 0.47},  // 0.1500
    {0.05, 0.05, 0.06, 0.06, 0.06, 0.10, 0.15, 0.19, 0.24, 0.27, 0.31, 0.34,
     0.38, 0.41, 0.43, 0.46},  // 0.2000
    {0.04, 0.04, 0.04, 0.05, 0.06, 0.09, 0.14, 0.18, 0.23, 0.25, 0.30, 0.34,
     0.37, 0.41, 0.43, 0.46},  // 0.2500
    {0.02, 0.02, 0.02, 0.04, 0.04, 0.07, 0.10, 0.14, 0.19, 0.20, 0.25, 0.27,
     0.30, 0.41, 0.43, 0.45},  // 0.2600
    {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01,
     0.01, 0.05, 0.05, 0.05}};  // 1.0000

double ARMLambda::GetRatioThreshold(const unsigned int num_dd,
                                     const double p_f_ILS) {
  // a more conservative control
  if (p_f_ILS >= 0.15) {
    return 30.0;
  } else if (p_f_ILS > 0.10) {
    return 20.0;
  } else if (p_f_ILS > 0.05) {
    return 10.0;
  }

  unsigned int i = 0;
  for (i = max_number_index - 1; i > 0; --i) {
    if (p_f_ILS <= ILS_failure_index[i] && p_f_ILS > ILS_failure_index[i - 1]) {
      break;
    }
  }
  double ratio_thres = 0.0;
  if (num_dd > max_number_dd_amb) {
    ratio_thres = 1.0 / ratio_look_up_table[i][max_number_dd_amb - 1];
  } else if (num_dd <= 3) {
    ratio_thres = 1.0 / ratio_look_up_table[i][3 - 1];
  } else {
    ratio_thres = 1.0 / ratio_look_up_table[i][num_dd - 1];
  }
  const double min_ratio = 2.5;
  if (ratio_thres <= min_ratio) {
    ratio_thres = min_ratio;
  }
  return ratio_thres;
}
bool ARMLambda::IsFixedSuccessfully(const double threshhold) {
  return (squared_ratio_ > threshhold) ? true : false;
}

double ARMLambda::GetRatio() { return squared_ratio_; }

double ARMLambda::GetUpperBoundFailureRate() {
  return upper_bound_failure_rate_ar_;
}

Eigen::MatrixXd ARMLambda::GetArZ() {
  return z_;
}

Eigen::MatrixXd ARMLambda::GetArQzz() {
  return qzz_;
}

double ARMLambda::Sign(double x) { return (x <= 0.0) ? -1.0 : 1.0; }

double ARMLambda::Round(double x) {
  return static_cast<int>(std::floor(x + 0.5));
}

void ARMLambda::SwapData(double* a, double* b) {
  double t(*a);
  *a = *b;
  *b = t;
}

double ARMLambda::GetGaussCdf(const double d) {
  const double a1 = 0.31938153;
  const double a2 = -0.356563782;
  const double a3 = 1.781477937;
  const double a4 = -1.821255978;
  const double a5 = 1.330274429;
  const double rsqrt_two_pi = 0.39894228040143267793994605993438;
  double k = 1.0 / (1.0 + 0.2316419 * fabs(d));
  double cnd = rsqrt_two_pi * exp(-0.5 * d * d) *
               (k * (a1 + k * (a2 + k * (a3 + k * (a4 + k * a5)))));
  if (d > 0) {
    cnd = 1.0 - cnd;
  }
  return cnd;
}

int ARMLambda::Factorize(const Eigen::MatrixXd &qxx,
                        Eigen::MatrixXd* low,
                        std::vector<double>* diag) {
  const int n = static_cast<int>(qxx.rows());
  Eigen::MatrixXd qcc_temp(qxx);
  (*low).resize(n, n);
  (*low).setZero();
  (*diag).resize(n, 0.0);
  for (int i = n - 1; i >= 0; i--) {
    (*diag)[i] = qcc_temp(i, i);
    if ((*diag)[i] <= 0.0) {
      return -1;
    }
    double temp = std::sqrt((*diag)[i]);
    for (int j = 0; j <= i; j++) {
      (*low)(i, j) = qcc_temp(i, j) / temp;
    }
    for (int j = 0; j <= i - 1; j++) {
      for (int k = 0; k <= j; k++) {
        qcc_temp(j, k) -= (*low)(i, k) * (*low)(i, j);
      }
    }
    for (int j = 0; j <= i; j++) {
      (*low)(i, j) /= (*low)(i, i);
    }
  }

  return 0;
}

void ARMLambda::Gauss(int i, int j, Eigen::MatrixXd* low,
                     Eigen::MatrixXd* z_int) {
  const int n = (*low).rows();
  const int mu = static_cast<int>(Round((*low)(i, j)));
  if (mu != 0) {
    for (int k = i; k < n; k++) {
      (*low)(k, j) -= static_cast<double>(mu) * (*low)(k, i);
    }
    for (int k = 0; k < n; k++) {
      (*z_int)(k, j) -= static_cast<double>(mu) * (*z_int)(k, i);
    }
  }
}

void ARMLambda::Permute(Eigen::MatrixXd* low, std::vector<double>* diag,
              int j, double del,
              Eigen::MatrixXd* z_int) {
  const int n = low->rows();
  double eta = (*diag)[j] / del;
  double lam = (*diag)[j + 1] * (*low)(j + 1, j) / del;
  (*diag)[j] = eta * (*diag)[j + 1];
  (*diag)[j + 1] = del;
  for (int k = 0; k <= j - 1; k++) {
    double a0 = (*low)(j, k);
    double a1 = (*low)(j + 1, k);
    (*low)(j, k) = -(*low)(j + 1, j) * a0 + a1;
    (*low)(j + 1, k) = eta * a0 + lam * a1;
  }
  (*low)(j + 1, j) = lam;
  for (int k = j + 2; k < n; k++) {
    SwapData(&(*low)(k, j), &(*low)(k, j + 1));
  }
  for (int k = 0; k < n; k++) {
    SwapData(&(*z_int)(k, j), &(*z_int)(k, j + 1));
  }
}

void ARMLambda::Reduct(Eigen::MatrixXd* low, std::vector<double>* diag,
                 Eigen::MatrixXd* z_int) {
  const int n = low->rows();
  int j(n - 2);
  int k(n - 2);
  while (j >= 0) {
    if (j <= k) {
      for (int i = j + 1; i < n; i++) {
        Gauss(i, j, low, z_int);
      }
    }
    double del = (*diag)[j] +
              (*low)(j + 1, j) * (*low)(j + 1, j) * (*diag)[j + 1];
    if (del + 1E-6 < (*diag)[j + 1]) {
      Permute(low, diag, j, del, z_int);
      k = j;
      j = n - 2;
    } else {
      j--;
    }
  }
}

int ARMLambda::SearchAmb(const Eigen::MatrixXd &L, const std::vector<double> &D,
                      const Eigen::MatrixXd &zs,
                      Eigen::MatrixXd* z_int,
                      std::vector<double>* square_res,
                      const int &m) {
  const int LOOPMAX = 10000;
  const int n = L.rows();

  z_int->resize(n, m);
  z_int->setZero();
  square_res->resize(m, 0.0);
  Eigen::MatrixXd S(n, n);
  S.setZero(n, n);
  std::vector<double> dist(n, 0.0);
  std::vector<double> zb(n, 0.0);
  std::vector<double> z(n, 0.0);
  std::vector<double> step(n, 0.0);

  int k = n - 1;
  dist[k] = 0.0;
  zb[k] = zs(k, 0);
  z[k] = Round(zb[k]);
  double y = zb[k] - z[k];
  step[k] = Sign(y);

  int c = 0;
  int nn = 0;
  int imax = 0;
  double maxdist = 1E99;
  for (c = 0; c < LOOPMAX; c++) {
    double newdist = dist[k] + y * y / D[k];
    if (newdist < maxdist) {
      if (k != 0) {
        dist[--k] = newdist;
        for (int i = 0; i <= k; i++) {
          S(k, i) = S(k + 1, i) + (z[k + 1] - zb[k + 1]) * L(k + 1, i);
        }
        zb[k] = zs(k, 0) + S(k, k);
        z[k] = Round(zb[k]);
        y = zb[k] - z[k];
        step[k] = Sign(y);
      } else {
        if (nn < m) {
          if (nn == 0 || newdist > (*square_res)[imax]) {
            imax = nn;
          }
          for (int i = 0; i < n; i++) {
            (*z_int)(i, nn) = z[i];
          }
          (*square_res)[nn++] = newdist;
        } else {
          if (newdist < (*square_res)[imax]) {
            for (int i = 0; i < n; i++) {
              (*z_int)(i, imax) = z[i];
            }
            (*square_res)[imax] = newdist;
            for (int i = imax = 0; i < m; i++) {
              if ((*square_res)[imax] < (*square_res)[i]) {
                imax = i;
              }
            }
          }
          maxdist = (*square_res)[imax];
        }
        z[0] += step[0];
        y = zb[0] - z[0];
        step[0] = -step[0] - Sign(step[0]);
      }
    } else {
      if (k == n - 1) {
        break;
      } else {
        k++;
        z[k] += step[k];
        y = zb[k] - z[k];
        step[k] = -step[k] - Sign(step[k]);
      }
    }
  }
  for (int i = 0; i < m - 1; i++) {
    for (int j = i + 1; j < m; j++) {
      if ((*square_res)[i] < (*square_res)[j]) {
        continue;
      }
      SwapData(&(*square_res)[i], &(*square_res)[j]);
      for (k = 0; k < n; k++) {
        SwapData(&(*z_int)(k, i), &(*z_int)(k, j));
      }
    }
  }
  if (c >= LOOPMAX) {
    // return -1;
  }
  return 0;
}

int ARMLambda::Lambda(const Eigen::MatrixXd& float_amb,
                     const Eigen::MatrixXd &q_nn,
                     Eigen::MatrixXd* fix_amb,
                     std::vector<double>* square_res,
                     const int &m) {
  if ((float_amb.rows() != q_nn.rows()) || (q_nn.rows() != q_nn.cols())) {
    return -1;
  }
  if (m < 1) {
    return -1;
  }
  const unsigned int n = static_cast<unsigned int>(float_amb.rows());
  Eigen::MatrixXd L(n, n);
  L.setZero(n, n);
  Eigen::MatrixXd E(n, m);
  E.setZero(n, m);

  std::vector<double> dd(n, 0.0);
  Eigen::MatrixXd z = Eigen::MatrixXd::Identity(n, n);
  // add a transformed Qxx to calculate the low-bound for AR success rate
  qzz_ = q_nn;
  square_res->resize(m, 0.0);
  if (Factorize(q_nn, &L, &dd) != 0) {
    return -1;
  }
  Reduct(&L, &dd, &z);
  Eigen::MatrixXd zta = z.transpose() * float_amb;
  z_ = z;
  qzz_ = z.transpose() * qzz_ * z;
  if (SearchAmb(L, dd, zta, &E, square_res, m) != 0) {
    return -1;
  }
  // Fix = Z'\E - Z nxn  E nxm Fix nxm
  try {
    Eigen::MatrixXd zi = z.inverse();
    *fix_amb = zi.transpose() * E;
  } catch (...) {
    return -1;
  }
  // get eigen value of transformed Qxx
  Eigen::EigenSolver<Eigen::MatrixXd> es(qzz_);
  std::complex<double> lam = es.eigenvalues()[0];
  double eigenvalue_max = lam.real();
  double eigenvalue_min = lam.real();
  for (unsigned int i = 1; i < n; ++i) {
    lam = es.eigenvalues()[i];
    if (eigenvalue_max < lam.real()) {
      eigenvalue_max = lam.real();
    }
    if (eigenvalue_min > lam.real()) {
      eigenvalue_min = lam.real();
    }
  }

  double lower_bound_success_rate_ar = 1.0;
  for (unsigned int i = 0; i < n; ++i) {
    lower_bound_success_rate_ar *= 2 * GetGaussCdf(0.5 / sqrt(dd[i])) - 1;
  }
  upper_bound_failure_rate_ar_ = 1 - lower_bound_success_rate_ar;
  return 0;
}

bool ARMLambda::ResolveIntegerAmbiguity(const Eigen::MatrixXd &amb_float,
                                         const Eigen::MatrixXd &amb_cov,
                                         Eigen::MatrixXd* amb_fix) {
  // check input
  if (amb_float.rows() != amb_cov.rows() ||
      amb_float.rows() != amb_cov.cols()) {
    return false;
  }
  std::vector<double> ss;
  if (Lambda(amb_float, amb_cov, amb_fix, &ss, 2) == 0) {
    if (ss.size() <= 0) {
      return false;
    }
    squared_ratio_ = (ss[0] < 1e-12) ? 9999.9 : ss[1] / ss[0];
    return true;
  }
  *amb_fix = amb_float;
  return false;
}

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
