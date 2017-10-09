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
 * Created by Matt Spraggs on 24/05/17
 */

#include <globals.hpp>
#include <serialisation.hpp>

#include "helpers.hpp"


TEST_CASE("Test serialise")
{
  ascendancy::Vec<2> v(0.1, 0.2);
  std::vector<char> buffer = ascendancy::serialise(1, 0.0, v);

  std::vector<int> expected_raw{
      0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x3f, -0x47, -0x67, -0x67, -0x67, -0x67, -0x67, -0x66, 0x3f, -0x37, -0x67,
      -0x67, -0x67, -0x67, -0x67, -0x66};
  const auto expected = ascendancy::static_cast_vector<char>(expected_raw);

  REQUIRE(buffer.size() == 28);
  REQUIRE(buffer == expected);
}


TEST_CASE("Test deserialise")
{
  std::vector<int> buffer_raw{
      0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x3f, -0x47, -0x67, -0x67, -0x67, -0x67, -0x67, -0x66, 0x3f, -0x37, -0x67,
      -0x67, -0x67, -0x67, -0x67, -0x66};
  const auto buffer = ascendancy::static_cast_vector<char>(buffer_raw);

  int i;
  double x;
  ascendancy::Vec<2> v;

  std::tie(i, x, v) =
      ascendancy::deserialise<int, double, ascendancy::Vec<2>>(buffer);

  REQUIRE(i == 1);
  REQUIRE(x == 0.0);
  REQUIRE(v[0] == Approx(0.1));
  REQUIRE(v[1] == Approx(0.2));
}