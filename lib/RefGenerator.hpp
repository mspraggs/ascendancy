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

#include "globals.hpp"


namespace ascendancy
{
  template <int NIn>
  class RefGenerator
  {
  public:

    RefGenerator(const unsigned int num_samples,
                 const unsigned int serialised_size);

    virtual ~RefGenerator() = default;

    virtual std::vector<char> serialise() const = 0;

    virtual void deserialise(const std::vector<char>& data) = 0;

    unsigned int serialised_size() const { return serialised_size_; }

    virtual Vec<NIn> get_reference(const unsigned int samp_num) const = 0;

    unsigned int get_num_samples() const { return num_samples_; }

  protected:
    unsigned int num_samples_, serialised_size_;
  };


  template <int NIn>
  RefGenerator<NIn>::RefGenerator(const unsigned int num_samples,
                                   const unsigned int serialised_size)
      : num_samples_(num_samples), serialised_size_(serialised_size)
  {
  }
}

#endif //ASCENDANCY_REFGENERATOR_HPP
