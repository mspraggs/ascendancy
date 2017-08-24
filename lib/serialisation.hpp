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
#ifndef ASCENDANCY_SERIALISATION_HPP
#define ASCENDANCY_SERIALISATION_HPP

#include <cstdint>
#include <vector>
#include <cstring>

#include "globals.hpp"
#include "utils.hpp"


namespace ascendancy
{
  template <typename It, typename T>
  void deserialise(const It begin, T& item);

  template <typename It, typename T, typename... Ts>
  void deserialise(const It begin, T& head, Ts&... tail);

  // The detail namespace contains implementation details that use a lot of
  // template meta-programming. Familiarity with C++11's variadic templates is
  // a must to understand the code here.
  namespace detail
  {
    bool is_big_endian();


    template<typename It>
    void byteswap(It buffer_begin, It buffer_end)
    {
      while (buffer_begin != buffer_end) {
        char store = *buffer_begin;
        *buffer_begin = *(buffer_end - 1);
        *(buffer_end - 1) = store;
        buffer_begin++;
        buffer_end--;
      }
    }


    template <bool value, typename T = void>
    using enable_if = typename std::enable_if<value, T>::type;


    template<typename T, typename... Ts>
    struct ParamPackSizer
    {
      static std::size_t size()
      { return sizeof(T) + ParamPackSizer<Ts...>::size(); }
    };


    template<typename T>
    struct ParamPackSizer<T>
    {
      static std::size_t size()
      { return sizeof(T); }
    };


    struct IteratorChecker
    {
      template <typename T>
      using sfinae_true = std::true_type;

      template <typename It>
      using iterator_category =
        typename std::iterator_traits<It>::iterator_category;

      template <typename T>
      static sfinae_true<iterator_category<T>> check(int);

      template <typename>
      static std::false_type check(...);
    };


    template <typename T>
    using is_iterable = decltype(IteratorChecker::check<T>(0));


    template <typename It, typename... Args, std::size_t... Ints>
    void deserialise(const It begin, std::tuple<Args...>& args,
                     const Seq<Ints...>&)
    {
      ascendancy::deserialise(begin, std::get<Ints>(args)...);
    }
  }


  template <typename T>
  void make_big_endian(T& value)
  {
    auto value_ptr = reinterpret_cast<char*>(&value);
    if (detail::is_big_endian()) {
      detail::byteswap(value_ptr, value_ptr + sizeof(T));
    }
  }


  template <typename It, typename T>
  auto serialise(It begin, const T& item)
    -> detail::enable_if<detail::is_iterable<It>::value>
  {
    std::size_t nbytes = sizeof(T);
    auto item_ptr = reinterpret_cast<const char*>(&item);
    std::copy(item_ptr, item_ptr + nbytes, begin);

    if (detail::is_big_endian()) {
      detail::byteswap(begin, begin + nbytes);
    }
  }


  template <typename It, int M, int N>
  auto serialise(It begin, const Mat<M, N>& value)
    -> detail::enable_if<detail::is_iterable<It>::value>
  {
    std::size_t scalar_size = sizeof(typename Mat<M, N>::Scalar);

    for (unsigned int i = 0; i < M; ++i) {
      for (unsigned int j = 0; j < N; ++j) {
        serialise(begin + scalar_size * (i * N + j), value(i, j));
      }
    }
  }


  template <typename It, typename T, typename... Ts>
  auto serialise(It begin, const T& head, const Ts&... tail)
    -> detail::enable_if<detail::is_iterable<It>::value>
  {
    std::size_t nbytes = sizeof(T);
    serialise(begin, head);
    serialise(begin + nbytes, tail...);
  }


  template <typename T, typename... Ts>
  std::vector<char> serialise(const T& head, const Ts&... tail)
  {
    std::vector<char> ret(detail::ParamPackSizer<T, Ts...>::size());
    serialise(ret.begin(), head, tail...);
    return ret;
  }


  template <typename It, typename T>
  void deserialise(It begin, T& value)
  {
    std::size_t nbytes = sizeof(T);
    auto value_ptr = reinterpret_cast<char*>(&value);
    std::copy(begin, begin + nbytes, value_ptr);

    if (detail::is_big_endian()) {
      detail::byteswap(value_ptr, value_ptr + nbytes);
    }
  }


  template <typename It, int M, int N>
  void deserialise(const It begin, Mat<M, N>& value)
  {
    std::size_t scalar_size = sizeof(typename Mat<M, N>::Scalar);

    for (unsigned int i = 0; i < M; ++i) {
      for (unsigned int j = 0; j < N; ++j) {
        deserialise(begin + scalar_size * (i * N + j), value(i, j));
      }
    }
  }


  template <typename It, typename T, typename... Ts>
  void deserialise(const It begin, T& head, Ts&... tail)
  {
    std::size_t nbytes = sizeof(T);
    deserialise(begin, head);
    deserialise(begin + nbytes, tail...);
  }


  template <typename... Args>
  std::tuple<Args...> deserialise(const std::vector<char>& buffer)
  {
    // Extracts big-endian packed binary data from buffer and loads it into a
    // tuple of the specified template types. If buffer.size() is not equal to
    // the total size of all types, the return values are default-constructed.

    std::tuple<Args...> ret;

    // Check buffer is big enough and bail out if it's not
    if (buffer.size() < detail::ParamPackSizer<Args...>::size()) {
      return std::tuple<Args...>();
    }

    detail::deserialise(buffer.begin(), ret,
                        make_int_seq<sizeof...(Args)>());

    return ret;
  }
}

#endif //ASCENDANCY_SERIALISATION_HPP
