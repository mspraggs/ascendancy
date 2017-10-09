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
 * Created by Matt Spraggs on 11/08/17
 */


#include <Algorithm.hpp>
#include <ConstRefGenerator.hpp>
#include <ControlSystem.hpp>


int main(char argc, char* argv[])
{
  const std::string log_file_path = argc == 2 ? argv[1] : "";

  constexpr unsigned int nin = 4, nout = 4, nsamples = 1000000;

  using namespace ascendancy;

  const auto src = [] (const unsigned int i) { return Vec<nin>::Zero(); };
  auto snk = [] (const unsigned int i, const Vec<nout>& data) {};

  ControlSystem<nin, nout> control_system("main", 1000);

  control_system.add_algorithm<Algorithm<nin, nout>>(0);
  control_system.set_ref_generator<ConstRefGenerator<nin>>(
      Vec<nin>::Zero(), nsamples);

  DataStore data;

  control_system.run(0, data, src, snk);
}
