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
 * Created by Matt Spraggs on 06/04/17
 */

#include "helpers.hpp"

#include <Algorithm.hpp>
#include <ControlSystem.hpp>
#include <DataStore.hpp>
#include <RefGenerator.hpp>


using namespace ascendancy;


class DummyAlgorithm : public Algorithm<10, 10>
{
public:
  class DummyController : public Controller<10, 10>
  {
  public:
    Vec<10> compute_output(const Vec<10>& input) override
    { return input; }
  };

  void setup(const DataStore& parameters) override
  {
    for (unsigned int i = 0; i < 10; ++i) {
      array_data[i] = 0.0;
    }
    total = parameters.get<double>();
    half_total = 0.0;
  }

  Controller<10, 10>* get_controller() override { return &controller_; }

  void remember(const Vec<10>& inputs, const unsigned int samp_num) override
  {
    if (samp_num < 10) {
      array_data[samp_num] = inputs[samp_num];
    }
    total += inputs[samp_num];
  }

  void finish() override
  {
    half_total = total / 2.0;
  }

  std::array<double, 10> array_data;
  double total;
  double half_total;

private:
  DummyController controller_;
};


class DummyRefGenerator : public ascendancy::RefGenerator<10>
{
public:
  DummyRefGenerator(const unsigned int num_samples)
      : RefGenerator(num_samples, 0)
  {}

  std::vector<unsigned char> serialise() const override
  { return std::vector<unsigned char>(); }

  void deserialise(const std::vector<unsigned char>& data) override {}

  Vec<10> get_reference(const unsigned int samp_num) const override
  { return ascendancy::Vec<10>::Zero(); }
};


struct MockSrcSnk
{
  MockSrcSnk()
  {
    inputs.fill(Vec<10>::Zero());
    outputs.fill(Vec<10>::Zero());
  }

  const Vec<10>& operator()(const unsigned int i) const { return inputs[i]; }
  void operator()(const unsigned int i, const Vec<10>& data)
  { outputs[i] = data; }

  std::array<Vec<10>, 10> inputs, outputs;
};


TEST_CASE("Test ControlSystem") {

  DataStore store;
  store.set(1.5);

  ControlSystem<10, 10> controller("main", 20);
  controller.add_algorithm<DummyAlgorithm>(1);

  MockSrcSnk src_sink;
  for (unsigned int i = 0; i < 10; ++i) {
    src_sink.inputs[i] = Vec<10>::Random();
  }

  std::array<double, 10> expected_array_data;
  for (unsigned int i = 0; i < 10; ++i) {
    expected_array_data[i] = src_sink.inputs[i][i];
  }

  double expected_total = std::accumulate(expected_array_data.begin(),
                                          expected_array_data.end(), -1.5);

  SECTION("Test run")
  {
    using Catch::Detail::Approx;

    controller.set_ref_generator<DummyRefGenerator>(10);

    controller.run(1, store, src_sink, src_sink);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    while (controller.is_busy()) ;

    auto& algorithm = controller.get_algorithm<DummyAlgorithm>(1);

    for (unsigned int i = 0; i < 10; ++i) {
      REQUIRE(-src_sink.inputs[i] == ascendancy::Approx(src_sink.outputs[i]));
    }

    REQUIRE(algorithm.total == Approx(-expected_total));
    REQUIRE(algorithm.half_total == Approx(-expected_total / 2.0));
    for (unsigned int i = 0; i < 10; ++i) {
      REQUIRE(algorithm.array_data[i] == Approx(-expected_array_data[i]));
    }
  }
}
