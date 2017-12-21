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
 * Created by Matt Spraggs on 07/04/17
 */
#ifndef ASCENDANCY_ALGORITHM_HPP
#define ASCENDANCY_ALGORITHM_HPP

#include "Controller.hpp"
#include "ControlSystem.hpp"
#include "globals.hpp"


namespace ascendancy
{
  template <unsigned int NIn, unsigned int NOut>
  class Algorithm
  {
    // This class defines a base class that can be passed to a ControlSystem
    // object. The latter can then use the former in its run_internal function.
  public:
    virtual ~Algorithm() = default;

    // Specifies the setup stage of the algorithm
    virtual void setup(const DataStore& parameters) {}

    virtual Vec<NIn> get_ref_correction(const unsigned int samp_num)
    { return Vec<NIn>::Zero(); }

    virtual void remember(const Vec<NIn>& data, const unsigned int samp_num) {}

    virtual Controller<NIn, NOut>* get_controller() { return nullptr; };

    // Specifies the finalisation step of the algorithm.
    virtual void finish() {}

    virtual void notify(const DataStore&) {}

    virtual void accept(std::unordered_map<std::string, DataStore>& data) const
    {}
  };
}

#endif //ASCENDANCY_ALGORITHM_HPP
