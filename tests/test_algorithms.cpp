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
 * Created by Matt Spraggs on 17/04/17
 */

#include <Algorithm.hpp>
#include <NonCausalAlgorithm.hpp>
#include <GenericLiftedAlgorithm.hpp>

#include "helpers.hpp"


double compute_reference(const unsigned int samp_num)
{
  return std::sin(8.0 * samp_num / 100);
}


class DummyController : public ascendancy::Controller<2, 1>
{
public:
  DummyController() { mapping_ = ascendancy::Mat<1, 2>::Constant(0.5); }

  ascendancy::Vec<1> compute_output(const ascendancy::Vec<2>& input) override
  {
    return mapping_ * input;
  }

private:
  ascendancy::Mat<1, 2> mapping_;
};


class SubjectAlgorithm : public ascendancy::Algorithm<1, 1>
{
public:
  void do_notify(const ascendancy::DataStore& data)
  {
    notify_observers(data);
  }

  void notify(const ascendancy::DataStore& data) override
  {
    if (data.has_value<int>()) {
      value_ = data.get<int>();
    }
  }

  int get_value() const { return value_; }

private:
  int value_;
};


TEST_CASE("Test Observer/Subject")
{
  using namespace ascendancy;

  SubjectAlgorithm algorithm1, algorithm2;

  int value = 0;

  auto observer_func =
      [&algorithm2, &value] (const DataStore& data) {
        value = data.get<int>();
        algorithm2.notify(data);
      };

  DataStore data;
  data.set(5);

  algorithm1.subscribe(0, observer_func);
  algorithm1.do_notify(data);

  REQUIRE(value == 5);
  REQUIRE(algorithm2.get_value() == 5);
}


TEST_CASE("Test NonCausalAlgorithm")
{
  using namespace ascendancy;

  constexpr unsigned int nstates= 2, nin = 2, nout = 1, nsamples = 10;

  const Mat<nstates, nstates> A = Mat<nstates, nstates>::Identity();
  const auto B = make_matrix<nstates, nin>({{0.3, 0.4}, {0.0, 1.0}});
  const auto C = make_matrix<nin, nstates>({{0.1, 0.0}, {1.0, 0.5}});

  const auto state_space = make_state_space(A, B, C);

  NonCausalAlgorithm<nin, nout> algorithm(state_space, nsamples);

  SECTION("Test remember")
  {
    for (unsigned int samp_num = 0; samp_num < nsamples; ++samp_num) {
      algorithm.remember(Vec<nin>::Random(), samp_num);
    }
  }

  SECTION("Test get_ref_correction")
  {
    for (unsigned int samp_num = 0; samp_num < nsamples; ++samp_num) {
      const auto correction = algorithm.get_ref_correction(samp_num);
      REQUIRE(correction == ascendancy::Approx(Vec<nin>::Zero()));
    }
  }

  SECTION("Test set/get_controller")
  {
    using Catch::Detail::Approx;

    REQUIRE(algorithm.get_controller() == nullptr);
    algorithm.set_controller<DummyController>();
    REQUIRE(algorithm.get_controller() != nullptr);

    auto controller_ptr = algorithm.get_controller();

    REQUIRE(controller_ptr->compute_output(Vec<2>::Ones())[0] == Approx(1.0));
  }

  SECTION("Test finish")
  {
    for (unsigned int samp_num = 0; samp_num < nsamples; ++samp_num) {
      algorithm.remember(Vec<nin>::Ones(), samp_num);
    }

    algorithm.finish();

    const auto expected_elem = make_matrix<nin, 1>({{0.07}, {1.2}});

    for (unsigned int i = 0; i < nsamples; ++i)
    {
      Vec<nin> expected = expected_elem * (nsamples - i);
      REQUIRE(algorithm.get_ref_correction(i) == ascendancy::Approx(expected));
    }
  }
}


TEST_CASE("Test GenericLiftedAlgorithm")
{
  using namespace ascendancy;

  constexpr unsigned int nin = 2, nout = 1, nsamples = 10;

  Mat<Dyn, Dyn> mapping = Mat<Dyn, Dyn>::Identity(nsamples * nin,
                                                  nsamples * nin);
  mapping.topRightCorner<nin, nin>() = Mat<nin, nin>::Identity();

  GenericLiftedAlgorithm<nin, nout> algorithm(mapping);

  SECTION("Test remember")
  {
    for (unsigned int samp_num = 0; samp_num < nsamples; ++samp_num) {
      algorithm.remember(Vec<nin>::Random(), samp_num);
    }
  }

  SECTION("Test get_ref_correction")
  {
    for (unsigned int samp_num = 0; samp_num < nsamples; ++samp_num) {
      const auto correction = algorithm.get_ref_correction(samp_num);
      REQUIRE(correction == ascendancy::Approx(Vec<nin>::Zero()));
    }
  }

  SECTION("Test set/get_controller")
  {
    using Catch::Detail::Approx;
    REQUIRE(algorithm.get_controller() == nullptr);
    algorithm.set_controller<DummyController>();
    REQUIRE(algorithm.get_controller() != nullptr);

    auto controller_ptr = algorithm.get_controller();

    REQUIRE(controller_ptr->compute_output(Vec<2>::Ones())[0] == Approx(1.0));
  }

  SECTION("Test finish")
  {
    using ascendancy::Approx;

    Vec<nin> fake_error = Vec<nin>::Random();
    for (unsigned int samp_num = 0; samp_num < nsamples; ++samp_num) {
      algorithm.remember(fake_error, samp_num);
    }

    algorithm.finish();

    REQUIRE(algorithm.get_ref_correction(0) == Approx(2 * fake_error));

    for (unsigned int samp_num = 1; samp_num < nsamples; ++samp_num) {
      REQUIRE(algorithm.get_ref_correction(samp_num) == Approx(fake_error));
    }
  }
}
