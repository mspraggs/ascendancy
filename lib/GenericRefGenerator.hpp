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
        : RefGenerator<NIn>(static_cast<unsigned int>(reference.size())),
          reference_(std::move(reference))
    {}

    std::vector<unsigned char> serialise() const override;

    void deserialise(const std::vector<unsigned char>& data) override;

    Vec<NIn> get_reference(const unsigned int samp_num) const override
    {
      return reference_.at(samp_num);
    }

  private:
    aligned_vector<Vec<NIn>> reference_;
  };


  template <unsigned int NIn>
  std::vector<unsigned char> GenericRefGenerator<NIn>::serialise() const
  {
    // Build a buffer of bytes representing the internal state of the class

    // FlatBuffers object that will do the hard work.
    flatbuffers::FlatBufferBuilder fbb;

    using Offsets = std::vector<flatbuffers::Offset<serialisation::Vector>>;
    Offsets vectors(this->num_samples_);
    
    for (unsigned int i = 0; i < this->num_samples_; ++i) {
      vectors[i] = serialisation::CreateVector(
          fbb, fbb.CreateVector(reference_[i].data(), NIn));
    }
    
    auto values = serialisation::CreateMatrix(fbb, fbb.CreateVector(vectors));

    serialisation::GenericRefGeneratorBuilder builder(fbb);
    builder.add_num_samples(this->num_samples_);
    builder.add_values(values);

    fbb.Finish(builder.Finish());

    // Return a std::vector
    return std::vector<unsigned char>(fbb.GetBufferPointer(),
                                      fbb.GetBufferPointer() + fbb.GetSize());
  }


  template <unsigned int NIn>
  void GenericRefGenerator<NIn>::deserialise(
      const std::vector<unsigned char>& data)
  {
    this->template verify_buffer<serialisation::GenericRefGenerator>(data);
    const auto refgen_data =
        this->template parse_buffer<serialisation::GenericRefGenerator>(data);

    const auto num_samples = refgen_data->num_samples();

    const auto values = refgen_data->values();

    if (values == nullptr or values->data() == nullptr or
        values->data()->Length() != num_samples) {
      throw std::invalid_argument("GenericRefGenerator unable to retrieve "
                                      "sample information from supplied data.");
    }

    aligned_vector<Vec<NIn>> reference(num_samples);

    for (unsigned int i = 0; i < num_samples; ++i) {
      reference[i] = vector_from_buffer<NIn>(values->data()->Get(i));
    }

    this->num_samples_ = num_samples;
    reference_ = std::move(reference);
  }
}

#endif //ASCENDANCY_GENERICREFGENERATOR_HPP
