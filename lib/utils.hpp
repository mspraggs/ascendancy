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

#ifndef ASCENDANCY_UTILS_HPP
#define ASCENDANCY_UTILS_HPP

#include <chrono>
#include <fstream>
#include <initializer_list>

#include "globals.hpp"


namespace ascendancy
{
  template<std::size_t... Ints> struct Seq {};


  template<std::size_t Size, std::size_t... Ints>
  struct SeqGen : SeqGen<Size - 1, Size - 1, Ints...>
  {
  };


  template<std::size_t... Ints>
  struct SeqGen<0, Ints...>
  {
    typedef Seq<Ints...> type;
  };


  template<std::size_t Size>
  using make_int_seq = typename SeqGen<Size>::type;


  std::chrono::nanoseconds compute_sleep_offset();

  template <int M, int N>
  Mat<M, N> make_matrix(
      std::initializer_list<std::initializer_list<double>> list)
  {
    Mat<M, N> ret;

    assert(list.size() == M);

    int i = 0;
    for (auto& sublist : list) {
      assert(sublist.size() == N);
      int j = 0;
      for (auto& elem : sublist) {
        ret(i, j) = elem;
        ++j;
      }
      ++i;
    }

    return ret;
  }


  template <int M, int N, typename T>
  Mat<M, N> matrix_from_buffer(const T* matrix)
  {
    Mat<M, N> mat;

    if (matrix->data() == nullptr) {
      throw std::invalid_argument("Cannot create matrix from buffer "
                                      "(no data supplied).");
    }

    if (matrix->data()->Length() != M) {
      throw std::invalid_argument(
          "Cannot create matrix from buffer "
              "(supplied data is the wrong shape).");
    }

    for (unsigned int i = 0; i < M; ++i) {
      const auto row = matrix->data()->Get(i);

      if (row->data() == nullptr) {
        throw std::invalid_argument("Cannot create matrix from buffer "
                                        "(row data is missing).");
      }

      if (row->data()->Length() != N) {
        throw std::invalid_argument(
            "Cannot create matrix from buffer "
                "(supplied data is the wrong shape).");
      }

      for (unsigned int j = 0; j < N; ++j) {
        mat(i, j) = row->data()->Get(j);
      }
    }

    return mat;
  }


  template <int N, typename T>
  Vec<N> vector_from_buffer(const T* vector)
  {
    Vec<N> vec;

    if (vector->data() == nullptr) {
      throw std::invalid_argument("Cannot create vector from buffer "
                                      "(no data supplied).");
    }

    if (vector->data()->Length() != N) {
      throw std::invalid_argument(
          "Cannot create vector from buffer "
              "(supplied data is the wrong shape).");
    }

    for (unsigned int i = 0; i < N; ++i) {
      vec[i] = vector->data()->Get(i);
    }

    return vec;
  }


  template <typename T>
  std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec)
  {
    for (auto it = vec.begin(); it != vec.end() - 1; ++it) {
      os << *it << ' ';
    }
    os << vec.back();
    return os;
  }


  template <typename T, std::size_t N>
  std::ostream& operator<<(std::ostream& os, const std::array<T, N>& arr)
  {
    for (auto it = arr.begin(); it != arr.end() - 1; ++it) {
      os << *it << ' ';
    }
    os << arr.back();
    return os;
  }


  template <typename T>
  std::istream& operator>>(std::istream& is, std::vector<T>& vec)
  {
    while (!is.eof()) {
      T value;
      is >> value;

      if (is.fail()) {
        break;
      }

      vec.push_back(value);
    }

    return is;
  }


  template <typename T, std::size_t N>
  std::istream& operator>>(std::istream& is, std::array<T, N>& arr)
  {
    for (auto i = 0; i < N and not is.eof(); ++i) {
      T value;
      is >> value;

      if (is.fail()) {
        break;
      }

      arr[i] = value;
    }

    return is;
  }


  template <int N>
  std::istream& operator>>(std::istream& is, Vec<N>& vec)
  {
    for (auto i = 0; i < N and not is.eof(); ++i) {
      typename Vec<N>::Scalar value;
      is >> value;

      if (is.fail()) {
        break;
      }

      vec[i] = value;
    }

    return is;
  }


  template <int M, int N>
  Mat<M, N> load_csv_matrix(std::ifstream& file)
  {
    Mat<M, N> ret = Mat<M, N>::Zero();
    std::string line;

    for (unsigned int i = 0; i < M; ++i) {
      std::getline(file, line);

      std::istringstream ss(line);
      std::string num;

      for (unsigned int j = 0; j < N; ++j) {
        std::getline(ss, num, ',');
        ret(i, j) = std::stod(num);
      }
    }

    return ret;
  }


  template<int M, int N>
  Mat<N, M> pinverse(const Mat<M, N>& mat, const double tolerance = -1.0)
  {
    double epsilon =
        (tolerance <= 0.0) ?
        std::min(M, N) * std::numeric_limits<double>::epsilon() :
        tolerance;

    Eigen::JacobiSVD<Mat<M, N>> svd(mat,
                                    Eigen::ComputeFullU | Eigen::ComputeFullV);

    auto& singular_values = svd.singularValues();
    Mat<N, M> singular_values_inv = Mat<N, M>::Zero();

    for (int i = 0; i < singular_values.size(); ++i) {
      singular_values_inv(i, i) =
          (singular_values(i) > epsilon) ? 1 / singular_values(i) : 0.0;
    }

    return svd.matrixV() * singular_values_inv * svd.matrixU().adjoint();
  }


  Mat<Dyn, Dyn> pinverse(const Mat<Dyn, Dyn>& mat,
                         const double tolerance = -1.0);
}

#endif //ASCENDANCY_UTILS_HPP
