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

#include <ConstRefGenerator.hpp>
#include <GenericRefGenerator.hpp>

#include "helpers.hpp"


TEST_CASE("Test ConstRefGenerator")
{
  using namespace ascendancy;

  Vec<2> value(3.2, 1.1);

  ConstRefGenerator<2> ref_generator(value, 100);

  SECTION("Test serialise")
  {
    const auto data = ref_generator.serialise();
    const auto expected = load_binary_data(TEST_DATA_PATH"/const_refgen.bin");
    REQUIRE(data == expected);
  }

  SECTION("Test deserialise")
  {
    Vec<2> ref = Vec<2>::Zero();
    ConstRefGenerator<2> ref_generator_local(ref, 1);
    const auto data = load_binary_data(TEST_DATA_PATH"/const_refgen.bin");

    ref_generator_local.deserialise(data);

    Vec<2> expected_ref(3.2, 1.1);

    REQUIRE(ref_generator_local.get_num_samples() == 100);
    REQUIRE(ref_generator_local.get_reference(0) ==
                ascendancy::Approx(expected_ref));
  }

  SECTION("Test get_reference")
  {
    REQUIRE(ref_generator.get_reference(0) == ascendancy::Approx(value));
  }
}


TEST_CASE("Test GenericRefGenerator")
{
  using namespace ascendancy;

  aligned_vector<Vec<2>> reference{Vec<2>(1.2, 2.2), Vec<2>(2.4, 2.8)};

  GenericRefGenerator<2> ref_generator(reference);

  SECTION("Test serialise")
  {
    const auto data = ref_generator.serialise();
    const auto expected = load_binary_data(TEST_DATA_PATH"/generic_refgen.bin");
    REQUIRE(data == expected);
  }

  SECTION("Test deserialise")
  {
    GenericRefGenerator<2> ref_generator_local;
    const auto data = load_binary_data(TEST_DATA_PATH"/generic_refgen.bin");

    ref_generator_local.deserialise(data);

    REQUIRE(ref_generator_local.get_num_samples() == 2);
    REQUIRE(ref_generator_local.get_reference(0) ==
                ascendancy::Approx(reference[0]));
    REQUIRE(ref_generator_local.get_reference(1) ==
                ascendancy::Approx(reference[1]));
  }
}
