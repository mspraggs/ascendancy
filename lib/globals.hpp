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
#ifndef ASCENDANCY_GLOBALS_HPP
#define ASCENDANCY_GLOBALS_HPP

#include <vector>

#include <Eigen/Dense>


namespace ascendancy
{
  constexpr double pi = 3.141592653589793;

  constexpr int Dyn = Eigen::Dynamic;

  template <int M, int N>
  using Mat = Eigen::Matrix<double, M, N>;

  template <int N>
  using Vec = Mat<N, 1>;

  template <typename T>
  using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;
}

#endif //ASCENDANCY_GLOBALS_HPP
