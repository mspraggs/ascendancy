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
 * Created by Matt Spraggs on 26/12/17
 */

#include "DataStore.hpp"


namespace ascendancy
{
  DataStore::DataStore(const DataStore& other)
      : mapping_(other.mapping_), data_(other.data_.size())
  {
    for (unsigned int i = 0; i < other.data_.size(); ++i) {
      data_[i].reset(other.data_[i]->clone());
    }
  }


  DataStore::DataStore(DataStore&& other) noexcept
      : mapping_(std::move(other.mapping_)), data_(std::move(other.data_))
  {
  }


  DataStore& DataStore::operator=(const DataStore& other)
  {
    if (&other != this) {
      mapping_ = other.mapping_;
      data_.resize(other.data_.size());

      for (unsigned int i = 0; i < data_.size(); ++i) {
        data_[i].reset(other.data_[i]->clone());
      }
    }

    return *this;
  }


  DataStore& DataStore::operator=(DataStore&& other)
  {
    mapping_ = std::move(other.mapping_);
    data_ = std::move(other.data_);
    return *this;
  }
}