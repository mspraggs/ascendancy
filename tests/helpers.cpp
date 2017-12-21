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
 * Created by Matt Spraggs on 21/12/17
 */

#include <fstream>
#include <vector>


namespace ascendancy
{
  std::vector<unsigned char> load_binary_data(const std::string& file_path)
  {
    std::ifstream file(file_path);

    if (not file.good()) {
      throw std::ios_base::failure("Unable to read supplied file.");
    }

    file.seekg(0, std::ios::end);
    const auto size = static_cast<unsigned long>(file.tellg());
    file.seekg(0, std::ios::beg);

    std::vector<unsigned char> ret(size);
    file.readsome(reinterpret_cast<char*>(ret.data()), size);

    return ret;
  }
}