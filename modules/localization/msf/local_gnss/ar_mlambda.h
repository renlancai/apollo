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

#ifndef MODULES_LOCALIZATION_MSF_LOCAL_GNSS_AR_MLAMBDA_H_
#define MODULES_LOCALIZATION_MSF_LOCAL_GNSS_AR_MLAMBDA_H_

#include <Eigen/Eigen>
#include <vector>

namespace apollo {
namespace localization {
namespace local_gnss {

// Reference to: T. Z. X.-W. Chang, X. Yang, “MLAMBDA: a modified LAMBDA
// method for integer least-squares estimation,” Journal of Geodesy, 2005

class ARMLambda {
 public:
  ARMLambda() : squared_ratio_(0.0) {
    lower_bound_failure_rate_ar_ = 0.0;
    upper_bound_failure_rate_ar_ = 0.0;
    z_.resize(0, 0);
    qzz_.resize(0, 0);
  }
  virtual ~ARMLambda() {}

 public:
  virtual bool ResolveIntegerAmbiguity(const Eigen::MatrixXd &amb_float,
                                         const Eigen::MatrixXd &amb_cov,
                                         Eigen::MatrixXd* amb_fix);

  bool IsFixedSuccessfully(const double threshhold = 4.0);

  double GetRatio();

  double GetUpperBoundFailureRate();

  Eigen::MatrixXd GetArZ();

  Eigen::MatrixXd GetArQzz();

  double GetRatioThreshold(const unsigned int num_dd,
                             const double p_f_ILS = 0.15);

  double Sign(double x);

  double Round(double x);

  void SwapData(double* a, double* b);

 private:
  // Qxx = L' * diag(D) * L
  int Factorize(const Eigen::MatrixXd &qxx, Eigen::MatrixXd* low,
                std::vector<double>* diag);

  void Gauss(int i, int j, Eigen::MatrixXd* low, Eigen::MatrixXd* z_int);

  void Permute(Eigen::MatrixXd* low, std::vector<double>* diag,
                int j, double del,
                Eigen::MatrixXd* z_int);

  void Reduct(Eigen::MatrixXd* low, std::vector<double>* diag,
                 Eigen::MatrixXd* z_int);

  // lambda /m-lambda SearchAmb
  virtual int SearchAmb(const Eigen::MatrixXd &L, const std::vector<double> &D,
                     const Eigen::MatrixXd &zs,
                     Eigen::MatrixXd* z_int,
                     std::vector<double>* square_res,
                     const int &m = 2);

  // lambda/m-lambda integer least-square estimation
  // float_amb   Float parameters (n x 1)
  // q_nn        Covariance matrix of float parameters (n x n)
  // fix_amb     Fixed solutions (n x m)
  // square_res  Sum of squared residuals of fixed solutions (1 x m)
  // m           Number of fixed solutions
  int Lambda(const Eigen::MatrixXd& float_amb, const Eigen::MatrixXd& q_nn,
             Eigen::MatrixXd* fix_amb, std::vector<double>* square_res,
             const int &m = 2);

  double GetGaussCdf(const double d);

 private:
  double squared_ratio_;
  double upper_bound_failure_rate_ar_;
  double lower_bound_failure_rate_ar_;
  Eigen::MatrixXd z_;
  Eigen::MatrixXd qzz_;
};

}  // namespace local_gnss
}  // namespace localization
}  // namespace apollo
#endif
