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
 * Created by Matt Spraggs on 16/08/17
 */
#ifndef ASCENDANCY_CONTROLLER_HPP
#define ASCENDANCY_CONTROLLER_HPP

#include "globals.hpp"


namespace ascendancy
{
  template <unsigned int NIn, unsigned int NOut>
  class Controller
  {
  public:
    virtual ~Controller() = default;

    virtual Vec<NOut> compute_output(const Vec<NIn>& input)
    { return Vec<NOut>::Zero(); }
  };
}

#endif //ASCENDANCY_CONTROLLER_HPP
