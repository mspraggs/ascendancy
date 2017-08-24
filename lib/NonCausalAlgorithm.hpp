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
 * Created by Matt Spraggs on 15/08/17
 */
#ifndef ASCENDANCY_NONCAUSALALGORITHM_HPP
#define ASCENDANCY_NONCAUSALALGORITHM_HPP

#include <Algorithm.hpp>
#include <StateSpace.hpp>


namespace ascendancy
{
  template <unsigned int NIn, unsigned int NOut>
  class NonCausalAlgorithm : public Algorithm<NIn, NOut>
  {
  public:
    template <unsigned int NStates>
    NonCausalAlgorithm(const StateSpace<NStates, NIn, NIn>& state_space,
                       const unsigned int num_samples);
    NonCausalAlgorithm(aligned_vector<Mat<NIn, NIn>> learning_elements);

    Vec<NIn> get_ref_correction(const unsigned int samp_num) override;

    void remember(const Vec<NIn>& data, const unsigned int samp_num) override;

    template <typename T, typename... Args>
    void set_controller(Args&&... args)
    {
      controller_.reset(new T(std::forward<Args>(args)...));
    }

    void set_controller(std::unique_ptr<Controller<NIn, NOut>> controller)
    {
      controller_ = std::move(controller);
    }

    Controller<NIn, NOut>* get_controller() override
    { return controller_.get(); }

    void finish() override;

  private:
    aligned_vector<Mat<NIn, NIn>> learning_elements_;
    aligned_vector<Vec<NIn>> corrections_, errors_;
    std::unique_ptr<Controller<NIn, NOut>> controller_;
  };


  template <unsigned int NIn, unsigned int NOut>
  template <unsigned int NStates>
  NonCausalAlgorithm<NIn, NOut>::NonCausalAlgorithm(
      const StateSpace<NStates, NIn, NIn>& state_space,
      const unsigned int num_samples)
      : learning_elements_(num_samples, Mat<NIn, NIn>::Zero()),
        corrections_(num_samples, Vec<NIn>::Zero()),
        errors_(num_samples, Vec<NIn>::Zero())
  {
    for (unsigned int i = 0; i < num_samples; ++i) {
      // Populate learning_elements_[i] with C A^i B
      learning_elements_[i] = state_space.compute_step_mapping(i + 1);
    }
  }


  template <unsigned int NIn, unsigned int NOut>
  NonCausalAlgorithm<NIn, NOut>::NonCausalAlgorithm(
      aligned_vector<Mat<NIn, NIn>> learning_elements)
      : learning_elements_(std::move(learning_elements)),
        corrections_(learning_elements.size(), Vec<NIn>::Zero()),
        errors_(learning_elements.size(), Vec<NIn>::Zero())
  {
  }


  template <unsigned int NIn, unsigned int NOut>
  Vec<NIn> NonCausalAlgorithm<NIn, NOut>::get_ref_correction(
      const unsigned int samp_num)
  {
    return corrections_.at(samp_num);
  }


  template <unsigned int NIn, unsigned int NOut>
  void NonCausalAlgorithm<NIn, NOut>::remember(
      const Vec<NIn>& data, const unsigned int samp_num)
  {
    errors_.at(samp_num) = data;
  }


  template <unsigned int NIn, unsigned int NOut>
  void NonCausalAlgorithm<NIn, NOut>::finish()
  {
    for (unsigned int i = 0; i < corrections_.size(); ++i) {
      corrections_[i] = Vec<NIn>::Zero();
      for (unsigned int j = i; j < errors_.size(); ++j) {
        corrections_[i] += learning_elements_[j - i] * errors_[j];
      }
    }
  }
}

#endif //ASCENDANCY_NONCAUSALALGORITHM_HPP
