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
 * Created by Matt Spraggs on 23/08/17
 */
#ifndef ASCENDANCY_PIDCONTROLLER_HPP
#define ASCENDANCY_PIDCONTROLLER_HPP

#include "Controller.hpp"


namespace ascendancy
{
  template <unsigned int NIn, unsigned int NOut>
  class PIDController : public Controller<NIn, NOut>
  {
  public:
    PIDController(const double kp, const double ki, const double kd)
    {
      static_assert(NIn == 1 and NOut == 1,
                    "Need NIn == 1 and NOut == 1 for scalar constructor");
      kp_.setConstant(kp);
      ki_.setConstant(ki);
      kd_.setConstant(kd);
      prev_err_ = Vec<NIn>::Zero();
      integral_ = Vec<NIn>::Zero();
    }

    PIDController(const Mat<NOut, NIn>& kp, const Mat<NOut, NIn>& ki,
                  const Mat<NOut, NIn>& kd)
        : kp_(kp), ki_(ki), kd_(kd)
    {
      prev_err_ = Vec<NIn>::Zero();
      integral_ = Vec<NIn>::Zero();
    }

    Vec<NOut> compute_output(const Vec<NIn>& input) override;

    void set_proportional_gain(const Mat<NOut, NIn>& kp) { kp_ = kp; }
    void set_integral_gain(const Mat<NOut, NIn>& ki) { ki_ = ki; }
    void set_derivative_gain(const Mat<NOut, NIn>& kd) { kd_ = kd; }

  private:
    Mat<NOut, NIn> kp_, ki_, kd_;
    Vec<NIn> prev_err_, integral_;
  };


  template <unsigned int NIn, unsigned int NOut>
  Vec<NOut> PIDController<NIn, NOut>::compute_output(const Vec<NIn>& input)
  {
    Vec<NIn> diff = input - prev_err_;
    integral_ += input;

    return kp_ * input + ki_ * integral_ + kd_ * diff;
  }
}

#endif //ASCENDANCY_PIDCONTROLLER_HPP
