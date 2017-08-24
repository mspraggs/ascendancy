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

#include "catch.hpp"

#include <PIDController.hpp>


TEST_CASE("Test PIDController")
{
  using namespace ascendancy;

  PIDController<1, 1> controller(1.0, 0.1, 0.01);

  SECTION("Test compute_action")
  {
    Vec<1> input = Vec<1>::Constant(0.5);

    Vec<1> output = controller.compute_output(input);

    REQUIRE(output[0] == Approx(0.555));
  }
}