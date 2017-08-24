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
 * Created by Matt Spraggs on 28/04/17
 */
#ifndef ASCENDANCY_MODEL_HPP
#define ASCENDANCY_MODEL_HPP

#include "globals.hpp"
#include "utils.hpp"


namespace ascendancy
{
  // Template enapsulating state-space representation of a LTI ODE with NStates
  // states, NIn inputs and NOut outputs
  template <unsigned int NStates, unsigned int NIn, unsigned int NOut>
  class StateSpace
  {
  public:
    StateSpace(const Mat<NStates, NStates>& A, const Mat<NStates, NIn>& B,
          const Mat<NOut, NStates>& C, const Mat<NOut, NIn>& D)
    : A_(A), B_(B), C_(C), D_(D)
    {
      x_.setZero();
    }

    StateSpace(const Mat<NStates, NStates>& A, const Mat<NStates, NIn>& B,
               const Mat<NOut, NStates>& C)
        : StateSpace(A, B, C, Mat<NOut, NIn>::Zero())
    {}

    void set_state(const Vec<NStates>& state) { x_ = state; }

    void reset() { set_state(Vec<NStates>::Zero()); }

    Vec<NOut> step(const Vec<NIn>& u);

    Mat<NOut, NIn> compute_step_mapping(const unsigned int num_steps) const;

  protected:
    Vec<NStates> x_;

    Mat<NStates, NStates> A_;
    Mat<NStates, NIn> B_;
    Mat<NOut, NStates> C_;
    Mat<NOut, NIn> D_;
  };


  template <unsigned int NStates, unsigned int NIn, unsigned int NOut>
  Vec<NOut> StateSpace<NStates, NIn, NOut>::step(const Vec<NIn>& u)
  {
    Vec<NIn> y = C_ * x_ + D_ * u;
    x_ = A_ * x_ + B_ * u;

    return y;
  }


  template <unsigned int NStates, unsigned int NIn, unsigned int NOut>
  Mat<NOut, NIn> StateSpace<NStates, NIn, NOut>::compute_step_mapping(
      const unsigned int num_steps) const
  {
    Mat<NOut, NStates> accumulation = C_;

    for (unsigned int i = 0; i < num_steps - 1; ++i) {
      accumulation *= A_;
    }

    return accumulation * B_;
  }


  template <int NStates, int NIn, int NOut>
  StateSpace<NStates, NIn, NOut> make_state_space(
      const Mat<NStates, NStates>& A, const Mat<NStates, NIn>& B,
      const Mat<NOut, NStates>& C,
      const Mat<NOut, NIn>& D = Mat<NOut, NIn>::Zero())
  {
    return StateSpace<NStates, NIn, NOut>(A, B, C, D);
  }
}

#endif //ASCENDANCY_MODEL_HPP
