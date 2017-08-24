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
 * Created by Matt Spraggs on 09/05/17
 */
#ifndef ASCENDANCY_LOGGER_HPP
#define ASCENDANCY_LOGGER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mutex>
#include <thread>


namespace ascendancy
{
  enum class LogLevel
  {
    Critical,
    Error,
    Warning,
    Info,
    Debug
  };


  template <typename T>
  void unpack_to_string(std::stringstream& ss, const T& p)
  {
    ss << p;
  }


  template <typename T, typename... Ts>
  void unpack_to_string(std::stringstream& ss, const T& p0, const Ts&... ps)
  {
    ss << p0;
    unpack_to_string(ss, ps...);
  }


  class Logger
  {
  public:
    Logger(const LogLevel max_level, const bool log_to_console,
           const std::string& log_file_path = "")
        : max_level_(max_level), console_out_(log_to_console),
          watch_queue_(true), queue_size_(0), queue_head_(0), queue_tail_(0)
    {
      if (log_file_path.size() > 0) {
        log_file_.open(log_file_path);
      }

      watching_thread_ = std::thread([this] () { watch_queue(); });

      write(LogLevel::Info, "Finished constructing Logger instance.");
    }

    ~Logger();

    void write(const LogLevel level, const std::string& message);

    template <typename... Ts>
    void write(const LogLevel level, const Ts&... params);

  private:
    using TimeType = std::chrono::time_point<std::chrono::system_clock>;
    using QueueElemType = std::tuple<std::string, LogLevel, TimeType>;

    void watch_queue();

    static std::string format_time(const TimeType& tpoint);

    LogLevel max_level_;
    bool console_out_;
    std::ofstream log_file_;

    std::thread watching_thread_;

    std::atomic_bool watch_queue_;
    static constexpr unsigned int max_queue_size_ = 10000;
    std::atomic<unsigned int> queue_size_, queue_head_, queue_tail_;
    std::array<QueueElemType, max_queue_size_> queue_;

    std::mutex mutex_;
    std::condition_variable queue_condition_;
  };


  template <typename... Ts>
  void Logger::write(const LogLevel level, const Ts&... params)
  {
    std::stringstream ss;
    unpack_to_string(ss, params...);
    write(level, ss.str());
  }
}

#endif //ASCENDANCY_LOGGER_HPP
