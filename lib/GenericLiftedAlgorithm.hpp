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

#ifndef ASCENDANCY_GENERICLIFTEDALGORITHM_HPP
#define ASCENDANCY_GENERICLIFTEDALGORITHM_HPP

#include "Algorithm.hpp"


namespace ascendancy
{
  template <unsigned int NIn, unsigned int NOut>
  class GenericLiftedAlgorithm : public Algorithm<NIn, NOut>
  {
  public:
    GenericLiftedAlgorithm(const Mat<Dyn, Dyn>& mapping);

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
    unsigned int num_samples_;
    aligned_vector<Mat<NIn, NIn>> mapping_;
    std::unique_ptr<Controller<NIn, NOut>> controller_;
    aligned_vector<Vec<NIn>> corrections_, errors_;
  };


  template <unsigned int NIn, unsigned int NOut>
  GenericLiftedAlgorithm<NIn, NOut>::GenericLiftedAlgorithm(
      const Mat<Dyn, Dyn>& mapping)
  {
    if (mapping.rows() % NIn != 0 or mapping.cols() % NIn != 0) {
      throw std::invalid_argument("Supplied mapping has invalid number of "
                                      "rows or columns");
    }

    if (mapping.rows() != mapping.cols()) {
      throw std::invalid_argument("Supplied mapping has inconsistent numbers "
                                      "of rows and columns");
    }

    num_samples_ = static_cast<unsigned int>(mapping.rows()) / NIn;
    errors_.resize(num_samples_, Vec<NIn>::Zero());
    corrections_.resize(num_samples_, Vec<NIn>::Zero());

    mapping_.resize(num_samples_ * num_samples_);

    for (unsigned int i = 0; i < num_samples_; ++i) {
      for (unsigned int j = 0; j < num_samples_; ++j) {
        mapping_[num_samples_ * i + j] =
            mapping.block<NIn, NIn>(i * NIn, j * NIn);
      }
    }
  }


  template <unsigned int NIn, unsigned int NOut>
  Vec<NIn> GenericLiftedAlgorithm<NIn, NOut>::get_ref_correction(
      const unsigned int samp_num)
  {
    return corrections_.at(samp_num);
  }


  template <unsigned int NIn, unsigned int NOut>
  void GenericLiftedAlgorithm<NIn, NOut>::remember(const Vec<NIn>& data,
                                                   const unsigned int samp_num)
  {
    errors_.at(samp_num) = data;
  }


  template <unsigned int NIn, unsigned int NOut>
  void GenericLiftedAlgorithm<NIn, NOut>::finish()
  {
    std::fill(corrections_.begin(), corrections_.end(), Vec<NIn>::Zero());

    for (unsigned int i = 0; i < num_samples_; ++i) {
      for (unsigned int j = 0; j < num_samples_; ++j) {
        corrections_[i] += mapping_[num_samples_ * i + j] * errors_[j];
      }
    }
  }
}

#endif //ASCENDANCY_GENERICLIFTEDALGORITHM_HPP
