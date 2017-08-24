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

#ifndef ASCENDANCY_CONSTREFGENERATOR_HPP
#define ASCENDANCY_CONSTREFGENERATOR_HPP

#include "globals.hpp"
#include "RefGenerator.hpp"
#include "serialisation.hpp"


namespace ascendancy
{
  template <unsigned int NIn>
  class ConstRefGenerator : public RefGenerator<NIn>
  {
  public:
    ConstRefGenerator(const Vec<NIn>& value, const unsigned int num_samples)
        : RefGenerator<NIn>(num_samples,
                            sizeof(unsigned int) + NIn * sizeof(double)),
          value_(value)
    {}

    std::vector<char> serialise() const override;

    void deserialise(const std::vector<char>& data) override;

    Vec<NIn> get_reference(const unsigned int samp_num) const override;

  private:
    Vec<NIn> value_;
  };


  template <unsigned int NIn>
  std::vector<char> ConstRefGenerator<NIn>::serialise() const
  {
    return ascendancy::serialise(this->num_samples_, value_);
  }


  template <unsigned int NIn>
  void ConstRefGenerator<NIn>::deserialise(const std::vector<char>& data)
  {
    if (data.size() != this->serialised_size_) {
      throw std::length_error("Supplied data to deserialise has bad length. "
                                  "ConstRefGenerator cannot deserialise.");
    }

    std::tie(this->num_samples_, value_) =
        ascendancy::deserialise<unsigned int, Vec<NIn>>(data);
  }


  template <unsigned int NIn>
  Vec<NIn> ConstRefGenerator<NIn>::get_reference(
      const unsigned int samp_num) const
  {
    return value_;
  }
}

#endif //ASCENDANCY_CONSTREFGENERATOR_HPP
