/* This file is part of Ascendancy++
 *
 * Copyright 2017 Matt Spraggs
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Created by Matt Spraggs on 06/04/17
 */

#define EIGEN_STACK_ALLOCATION_LIMIT 0

#include <thread>

#include "utils.hpp"


namespace ascendancy {

  std::chrono::nanoseconds compute_sleep_offset()
  {
    long int total_nanosecs = 0;
    unsigned int num_samples = 1000;
    for (unsigned int i = 0; i < num_samples; ++i) {
      auto initial = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(std::chrono::nanoseconds(0));
      auto final = std::chrono::steady_clock::now();
      std::chrono::nanoseconds period = final - initial;
      total_nanosecs += period.count();
    }

    return std::chrono::nanoseconds(total_nanosecs / num_samples);
  }


  Mat<Dyn, Dyn> pinverse(const Mat<Dyn, Dyn>& mat, const double tolerance)
  {
    long min_dim = std::min(mat.rows(), mat.cols());
    double epsilon =
        (tolerance <= 0.0) ?
        min_dim * std::numeric_limits<double>::epsilon() : tolerance;

    Eigen::JacobiSVD<Mat<Dyn, Dyn>> svd(
        mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

    auto& singular_values = svd.singularValues();
    Mat<Dyn, Dyn> singular_values_inv = Mat<Dyn, Dyn>::Zero(min_dim, min_dim);

    for (int i = 0; i < singular_values.size(); ++i) {
      singular_values_inv(i, i) =
          (singular_values(i) > epsilon) ? 1 / singular_values(i) : 0.0;
    }

    return svd.matrixV() * singular_values_inv * svd.matrixU().adjoint();
  }
}