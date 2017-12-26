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
#ifndef ASCENDANCY_DATASTORE_HPP
#define ASCENDANCY_DATASTORE_HPP

#include <typeindex>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>


namespace ascendancy
{
  class DataStore
  {
  private:
    class ContainerBase
    {
    public:
      virtual ~ContainerBase() = default;

      virtual ContainerBase* clone() = 0;

      virtual void* get_ptr() = 0;
      virtual const void* get_ptr() const = 0;
    };

    template <typename T>
    class Container : public ContainerBase
    {
    public:
      Container(T value) : value_(std::move(value)) {}

      ContainerBase* clone() override { return new Container<T>(value_); }

      void* get_ptr() override { return static_cast<void*>(&value_); }

      const void* get_ptr() const override
      { return static_cast<const void*>(&value_); }

    private:
      T value_;
    };

  public:
    DataStore() = default;
    DataStore(const DataStore& other);
    DataStore(DataStore&& other) noexcept;

    DataStore& operator=(const DataStore& other);
    DataStore& operator=(DataStore&& other);

    template <typename T>
    bool has_value() const
    { return mapping_.count(std::type_index(typeid(T))) == 1; }
    template <typename T>
    void set(T value);
    template <typename T>
    const T& get() const;
    template <typename T>
    void unset();

  private:
    std::unordered_map<std::type_index, std::size_t> mapping_;
    std::vector<std::unique_ptr<ContainerBase>> data_;
  };


  template<typename T>
  void DataStore::set(T value)
  {
    const auto type_index = std::type_index(typeid(T));

    if (mapping_.count(type_index) == 0) {
      data_.emplace_back(new Container<T>(std::move(value)));
      mapping_[type_index] = data_.size() - 1;
    }
    else {
      const std::size_t index = mapping_[type_index];
      data_[index].reset(new Container<T>(std::move(value)));
    }
  }


  template<typename T>
  const T& DataStore::get() const
  {
    const auto type_index = std::type_index(typeid(T));
    const std::size_t index = mapping_.at(type_index);
    return *static_cast<const T*>(data_[index]->get_ptr());
  }


  template<typename T>
  void DataStore::unset()
  {
    const auto type_index = std::type_index(typeid(T));
    const std::size_t index = mapping_.at(type_index);
    data_.erase(data_.begin() + index);
    mapping_.erase(type_index);
  }
}

#endif //ASCENDANCY_DATASTORE_HPP
