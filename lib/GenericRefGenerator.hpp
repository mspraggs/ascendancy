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
 * Created by Matt Spraggs on 24/08/17
 */

#ifndef ASCENDANCY_GENERICREFGENERATOR_HPP
#define ASCENDANCY_GENERICREFGENERATOR_HPP

#include "globals.hpp"
#include "RefGenerator.hpp"
#include "serialisation.hpp"


namespace ascendancy
{
  template <unsigned int NIn>
  class GenericRefGenerator : public RefGenerator<NIn>
  {
  public:
    GenericRefGenerator(aligned_vector<Vec<NIn>> reference)
        : RefGenerator<NIn>(
            static_cast<unsigned int>(reference.size()),
            sizeof(unsigned int) + sizeof(double) * NIn * reference.size()),
          reference_(std::move(reference))
    {}

    std::vector<char> serialise() const override;

    void deserialise(const std::vector<char>& data) override;

    Vec<NIn> get_reference(const unsigned int samp_num) const override
    {
      return reference_.at(samp_num);
    }

  private:
    aligned_vector<Vec<NIn>> reference_;
  };


  template <unsigned int NIn>
  std::vector<char> GenericRefGenerator<NIn>::serialise() const
  {
    std::vector<char> ret(this->serialised_size_);

    ascendancy::serialise(ret.begin(), this->num_samples_);

    constexpr std::size_t sample_size = sizeof(double) * NIn;
    const auto offset = sizeof(unsigned int);

    for (unsigned int i = 0; i < reference_.size(); ++i) {
      ascendancy::serialise(ret.begin() + offset + sample_size * i,
                            reference_[i]);
    }

    return ret;
  }


  template <unsigned int NIn>
  void GenericRefGenerator<NIn>::deserialise(const std::vector<char>& data)
  {
    const std::string throw_msg =
        "Supplied data to deserialise has bad length. "
            "GenericRefGenerator cannot deserialise.";

    if (data.size() < sizeof(unsigned int)) {
      throw std::length_error(throw_msg);
    }

    const auto sample_data_size = data.size() - sizeof(unsigned int);

    unsigned int num_samples;
    std::tie(num_samples) = ascendancy::deserialise<unsigned int>(data);

    if (sample_data_size != NIn * sizeof(double) * num_samples) {
      throw std::length_error(throw_msg);
    }

    this->num_samples_ = num_samples;
    reference_.resize(this->num_samples_);

    constexpr std::size_t sample_size = sizeof(double) * NIn;
    constexpr std::size_t offset = sizeof(unsigned int);

    for (unsigned int i = 0; i < reference_.size(); ++i) {
      ascendancy::deserialise(data.begin() + offset + i * sample_size,
                              reference_[i]);
    }

    this->serialised_size_ =
        sizeof(unsigned int) + sizeof(double) * NIn * this->num_samples_;
  }
}

#endif //ASCENDANCY_GENERICREFGENERATOR_HPP
