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
 * Created by Matt Spraggs on 16/05/17
 */
#ifndef ASCENDANCY_REFGENERATOR_HPP
#define ASCENDANCY_REFGENERATOR_HPP

#include <typeindex>
#include <unordered_map>
#include <vector>

#include <flatbuffers/flatbuffers.h>

#include "globals.hpp"


namespace ascendancy
{
  template <int NIn>
  class RefGenerator
  {
  public:
    RefGenerator(const unsigned int num_samples);
    RefGenerator() : RefGenerator(0) {}

    virtual ~RefGenerator() = default;

    virtual std::vector<unsigned char> serialise() const = 0;

    virtual void deserialise(const std::vector<unsigned char>& data) = 0;

    virtual Vec<NIn> get_reference(const unsigned int samp_num) const = 0;

    unsigned int get_num_samples() const { return num_samples_; }

  protected:
    template <typename T>
    void verify_buffer(const std::vector<unsigned char>& buffer) const;

    template <typename T>
    const T* parse_buffer(const std::vector<unsigned char>& buffer) const;

    unsigned int num_samples_;
  };


  template <int NIn>
  RefGenerator<NIn>::RefGenerator(const unsigned int num_samples)
      : num_samples_(num_samples)
  {
  }

  template<int NIn>
  template <typename T>
  void RefGenerator<NIn>::verify_buffer(
      const std::vector<unsigned char>& buffer) const
  {
    // Use FlatBuffers to validate the supplied buffer using the specified
    // serialised flatbuffer type

    auto verifier = flatbuffers::Verifier(buffer.data(), buffer.size());
    const bool valid_buffer = verifier.VerifyBuffer<T>(nullptr);

    if (not valid_buffer) {
      throw std::invalid_argument("Binary data supplied to RefGenerator class "
                                      "is invalid.");
    }
  }


  template<int NIn>
  template<typename T>
  const T* RefGenerator<NIn>::parse_buffer(
      const std::vector<unsigned char>& buffer) const
  {
    return flatbuffers::GetRoot<T>(buffer.data());
  }
}

#endif //ASCENDANCY_REFGENERATOR_HPP
