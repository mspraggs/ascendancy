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
#include "ref_generators_generated.h"
#include "utils.hpp"


namespace ascendancy
{
  template <unsigned int NIn>
  class ConstRefGenerator : public RefGenerator<NIn>
  {
  public:
    ConstRefGenerator(const Vec<NIn>& value, const unsigned int num_samples)
        : RefGenerator<NIn>(num_samples), value_(value)
    {}
    ConstRefGenerator() : ConstRefGenerator(Vec<NIn>::Zero(), 0) {}

    std::vector<unsigned char> serialise() const override;

    void deserialise(const std::vector<unsigned char>& data) override;

    Vec<NIn> get_reference(const unsigned int) const override;

  private:
    Vec<NIn> value_;
  };


  template <unsigned int NIn>
  std::vector<unsigned char> ConstRefGenerator<NIn>::serialise() const
  {
    // Build a buffer of bytes representing the internal state of the class.

    // FlatBuffers object that will do the hard work.
    flatbuffers::FlatBufferBuilder fbb;

    // Create the constant used as the reference.
    auto value =
        serialisation::CreateVector(fbb, fbb.CreateVector(value_.data(), NIn));

    // Build the class buffer from the constant above and the number of samples
    serialisation::ConstRefGeneratorBuilder builder(fbb);
    builder.add_num_samples(this->num_samples_);
    builder.add_value(value);

    fbb.Finish(builder.Finish());

    // Return a std::vector
    return std::vector<unsigned char>(fbb.GetBufferPointer(),
                                      fbb.GetBufferPointer() + fbb.GetSize());
  }


  template <unsigned int NIn>
  void ConstRefGenerator<NIn>::deserialise(
      const std::vector<unsigned char>& data)
  {
    this->template verify_buffer<serialisation::ConstRefGenerator>(data);
    const auto refgen_data =
        this->template parse_buffer<serialisation::ConstRefGenerator>(data);

    const auto num_samples = refgen_data->num_samples();

    if (refgen_data->value() == nullptr) {
      throw std::invalid_argument("ConstRefGenerator unable to retrieve values "
                                      "from supplied buffer.");
    }

    value_ = vector_from_buffer<NIn>(refgen_data->value());
    this->num_samples_ = num_samples;
  }


  template <unsigned int NIn>
  Vec<NIn> ConstRefGenerator<NIn>::get_reference(const unsigned int) const
  {
    return value_;
  }
}

#endif //ASCENDANCY_CONSTREFGENERATOR_HPP
