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
 * Created by Matt Spraggs on 31/05/17
 */
#ifndef ASCENDANCY_TESTREFGEN_HPP
#define ASCENDANCY_TESTREFGEN_HPP

#include <RefGenerator.hpp>


namespace ascendancy
{
  template <int NOut>
  class TestRefGen : public RefGenerator<NOut>
  {
  public:
    TestRefGen(const unsigned int num_samples)
        : ascendancy::RefGenerator<NOut>(0, num_samples, 0)
    {}

    ascendancy::Vec<NOut> get_reference(
        const unsigned int samp_num) const override
    {
      return ascendancy::Vec<NOut>::Ones();
    }

    std::vector<char> serialise() const override { return std::vector<char>(); }

    void deserialise(const std::vector<char>& data) override {}
  };
}


#endif //ASCENDANCY_TESTREFGEN_HPP
