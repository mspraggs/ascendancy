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
 * Created by Matt Spraggs on 14/08/17
 */

#include <StateSpace.hpp>

#include "catch.hpp"


TEST_CASE("Test StateSpace")
{
  using namespace ascendancy;

  const auto A = make_matrix<2, 2>({{-0.8, -0.22}, {1.0, 0.0}});
  const auto B = make_matrix<2, 1>({{0.5}, {1.0}});
  const auto C = make_matrix<1, 2>({{1.0, 0.5}});

  StateSpace<2, 1, 1> model(A, B, C);

  SECTION("Test step")
  {
    const auto state = make_matrix<2, 1>({{0.3}, {0.4}});
    model.set_state(state);

    const auto input = make_matrix<1, 1>({{1.0}});

    auto output = model.step(input);
    REQUIRE(output[0] == Approx(0.5));

    output = model.step(input);
    REQUIRE(output[0] == Approx(0.822));
  }
}