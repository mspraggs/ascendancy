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

#ifndef ASCENDANCY_HELPERS_HPP
#define ASCENDANCY_HELPERS_HPP

#include <fstream>
#include <iostream>

#include <globals.hpp>

#include "catch.hpp"


namespace ascendancy
{
  template <typename T>
  using remove_reference_t =
      typename std::remove_reference<typename std::remove_const<T>::type>::type;


  template <typename MatType>
  class ApproxImpl
  {
    using ArrType =
        remove_reference_t<decltype(std::declval<MatType>().array())>;

  public:
    ApproxImpl(const MatType& value)
        : epsilon_(std::numeric_limits<float>::epsilon() * 100), margin_(0.0),
          value_(value)
    {}

    template <typename T,
              typename = typename std::enable_if<
                  std::is_constructible<MatType, T>::value>::type>
    friend bool operator==(const T& lhs, const ApproxImpl<MatType>& rhs)
    {
      const MatType abs_diff = (rhs.value_ - lhs).cwiseAbs();
      const MatType rhs_value =
          rhs.epsilon_ * rhs.value_.cwiseAbs() + rhs.margin_ * MatType::Ones();
      const MatType lhs_value =
          rhs.epsilon_ * lhs.cwiseAbs() + rhs.margin_ * MatType::Ones();

      const bool rhs_good = (abs_diff.array() <= rhs_value.array()).all();
      const bool lhs_good = (abs_diff.array() <= lhs_value.array()).all();

      return rhs_good and lhs_good;
    }

    template <typename T,
        typename = typename std::enable_if<
            std::is_constructible<MatType, T>::value>::type>
    friend bool operator==(const ApproxImpl<MatType>& lhs, const T& rhs)
    {
      return operator==(rhs, lhs);
    }

    template <typename T,
        typename = typename std::enable_if<
            std::is_constructible<MatType, T>::value>::type>
    friend bool operator!=(const T& lhs, const ApproxImpl<MatType>& rhs)
    {
      return not operator==(lhs, rhs);
    }

    template <typename T,
        typename = typename std::enable_if<
            std::is_constructible<MatType, T>::value>::type>
    friend bool operator!=(const ApproxImpl<MatType>& lhs, const T& rhs)
    {
      return operator!=(rhs, lhs);
    }

  private:
    typename MatType::RealScalar epsilon_, margin_;
    MatType value_;
  };


  template <typename ExprType>
  auto Approx(const ExprType& value)
      -> ApproxImpl<remove_reference_t<decltype(value.eval())>>
  {
    using MatType = remove_reference_t<decltype(value.eval())>;
    return ApproxImpl<MatType>(static_cast<MatType>(value));
  }


  template<int M, int N>
  Mat<M, N> load_matrix_from_file(const std::string& file_path)
  {
    ascendancy::Mat<M, N> ret;
    std::ifstream file(file_path);

    try {
      for (unsigned int i = 0; i < M; ++i) {
        std::string data;
        for (unsigned int j = 0; j < N - 1; ++j) {
          std::getline(file, data, ',');
          ret(i, j) = std::atof(data.c_str());
        }
        std::getline(file, data, '\n');
        ret(i, N - 1) = std::atof(data.c_str());
      }
    }
    catch (std::ios_base::failure& e) {
      std::cout << "Error reading from file: " << e.what() << std::endl;
    }

    return ret;
  }
}
#endif //ASCENDANCY_HELPERS_HPP
