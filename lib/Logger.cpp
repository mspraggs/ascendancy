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

#include <iomanip>

#include "Logger.hpp"


namespace ascendancy
{
  Logger::Logger(Logger&& logger) noexcept
      : max_level_(logger.max_level_), console_out_(logger.console_out_)
  {
    // First shut down the thread watching the queue for thread safety.
    logger.watch_queue_ = false;
    logger.write(LogLevel::Info, "Moving Logger instance...");
    if (logger.watching_thread_.joinable()) {
      logger.watching_thread_.join();
    }

    log_file_ = std::move(logger.log_file_);
    watch_queue_ = true;
    queue_size_ = logger.queue_size_.load();
    queue_head_ = logger.queue_head_.load();
    queue_tail_ = logger.queue_tail_.load();
    queue_ = logger.queue_;

    watching_thread_ = std::thread([this] () { watch_queue(); });

    write(LogLevel::Info, "Finished constructing Logger instance.");
  }


  void Logger::write(const ascendancy::LogLevel level,
                     const std::string& message)
  {
    if (level > max_level_) {
      return;
    }

    if (queue_size_ == max_queue_size_) {
      throw std::overflow_error("Log queue has overflown!");
    }

    std::lock_guard<std::mutex> lock(mutex_);

    queue_[queue_tail_] =
        std::forward_as_tuple(message, level,
                              std::chrono::system_clock::now());
    queue_tail_ = (queue_tail_ + 1) % max_queue_size_;
    queue_size_++;

    queue_condition_.notify_one();
  }


  Logger::~Logger()
  {
    watch_queue_ = false;
    write(LogLevel::Info, "Destroying Logger instance.");

    try {
      watching_thread_.join();
    }
    catch (const std::system_error& e) {
      write(LogLevel::Error, "Unable to join Logger watching thread: ",
            e.what());
    }
  }


  void Logger::watch_queue()
  {
    namespace ch = std::chrono;

    while (watch_queue_ or queue_size_ > 0) {

      std::unique_lock<std::mutex> lock(mutex_);
      queue_condition_.wait(lock, [this] { return queue_size_ > 0; });
      auto current_message = queue_[queue_head_];
      lock.unlock();
      
      queue_head_ = (queue_head_ + 1) % max_queue_size_;

      std::string prefix = "";

      switch (std::get<1>(current_message)) {
        case LogLevel::Debug:
          prefix = "DEBUG";
          break;
        case LogLevel::Info:
          prefix = "INFO";
          break;
        case LogLevel::Warning:
          prefix = "WARNING";
          break;
        case LogLevel::Error:
          prefix = "ERROR";
          break;
        case LogLevel::Critical:
          prefix = "CRITICAL";
          break;
      }

      auto time_str = format_time(std::get<2>(current_message));

      std::stringstream log_message;
      log_message << prefix << " @ " << time_str << ": "
                  << std::get<0>(current_message);

      if (console_out_) {
        auto& stream = (std::get<1>(current_message) < LogLevel::Info)
                       ? std::cerr : std::cout;
        stream << log_message.str() << std::endl;
      }

      log_file_ << log_message.str() << std::endl;

      queue_size_--;
    }
  }


  std::string Logger::format_time(const Logger::TimeType& tpoint)
  {
    namespace ch = std::chrono;
    auto millis =
        ch::duration_cast<ch::microseconds>(tpoint.time_since_epoch()).count();

    std::time_t ct = ch::system_clock::to_time_t(tpoint);

    char buf[100];
    std::strftime(buf, 100, "%Y-%m-%d %H:%M:%S", std::localtime(&ct));

    std::stringstream ss;
    ss << buf << '.' << std::setw(6) << std::setfill('0') << millis % 1000000;

    return ss.str();
  }
}