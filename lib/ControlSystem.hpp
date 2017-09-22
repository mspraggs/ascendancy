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
#ifndef ASCENDANCY_CONTROLSYSTEM_HPP
#define ASCENDANCY_CONTROLSYSTEM_HPP

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>

#include <sys/mman.h>

#include "DataStore.hpp"
#include "globals.hpp"
#include "Logger.hpp"
#include "RefGenerator.hpp"
#include "utils.hpp"


namespace ascendancy
{
  template <unsigned int NIn, unsigned int NOut>
  class Algorithm;

  template <unsigned int NIn, unsigned int NOut>
  class ControlSystem
  {
  public:
    constexpr static unsigned int num_inputs = NIn;
    constexpr static unsigned int num_outputs = NOut;

    // Define convenience type for controller input
    using InputType = Vec<NIn>;
    using OutputType = Vec<NOut>;

    ControlSystem(Logger& logger, const unsigned int frequency)
        : thread_running_(false), dirty_(false), logger_(&logger),
          frequency_(frequency)
    {
      logger_->write(LogLevel::Info, "Finished constructing ControlSystem "
          "instance.");

      try {
        algorithms_.reserve(100);
      }
      catch (const std::bad_alloc& e) {
        logger_->write(LogLevel::Warning, "Unable to reserve algorithms_: ",
                       e.what());
      }
    }

    ControlSystem(ControlSystem&& control_system) noexcept;
    ControlSystem(const ControlSystem& control_system) = delete;

    ~ControlSystem();

    ControlSystem& operator=(const ControlSystem& control_system) = delete;
    ControlSystem& operator=(ControlSystem&& control_system) = delete;

    template <typename Src, typename Snk>
    bool run(const unsigned int algorithm_id, const DataStore& parameters,
             Src& src, Snk& sink);
    void stop() { thread_running_ = false; }

    bool is_busy() const { return thread_running_; }

    // Variable setters
    template <typename T, typename... Args>
    void add_algorithm(unsigned int id, Args&&... args);
    void add_algorithm(unsigned int id,
                       std::unique_ptr<Algorithm<NIn, NOut>> algorithm);
    template <typename T, typename... Args>
    void set_ref_generator(Args&&... args);
    void set_ref_generator(std::unique_ptr<RefGenerator<NIn>> ref_generator);

    template <typename T>
    T& get_algorithm(const unsigned int index)
    {
      return *static_cast<T*>(algorithms_[algorithm_mapping_.at(index)].get());
    }
    template <typename T>
    const T& get_algorithm(const unsigned int index) const
    {
      return *static_cast<T*>(algorithms_[algorithm_mapping_.at(index)].get());
    }
    template <typename T = RefGenerator<NIn>>
    T& get_ref_generator()
    { return *static_cast<T*>(ref_generator_.get()); }
    template <typename T = RefGenerator<NIn>>
    const T& get_ref_generator() const
    { return *static_cast<T*>(ref_generator_.get()); }

    unsigned int get_frequency() const { return frequency_; }

    bool has_valid_state() const { return not dirty_; }

    void erase_algorithm(const unsigned int id);

  private:
    static void stack_prefault();

    template <typename Src, typename Snk>
    void run_internal(const std::size_t algorithm_idx,
                      const DataStore& parameters,
                      Src& src, Snk& sink);

    std::atomic_bool thread_running_, dirty_;
    std::thread rt_thread_;
    std::promise<void> wakeup_promise_;

    std::mutex global_mutex_;
    std::mutex input_mutex_;

    std::unordered_map<unsigned int, std::size_t> algorithm_mapping_;
    // Here are pointers to instances of the algorithm class, which effectively
    // implements the Pimpl idiom (they point to implementation)
    std::vector<std::unique_ptr<Algorithm<NIn, NOut>>> algorithms_;
    std::unique_ptr<RefGenerator<NIn>> ref_generator_;

    Logger* logger_;

    unsigned int frequency_;
  };


  template <unsigned int NIn, unsigned int NOut>
  constexpr unsigned int ControlSystem<NIn, NOut>::num_inputs;


  template <unsigned int NIn, unsigned int NOut>
  constexpr unsigned int ControlSystem<NIn, NOut>::num_outputs;


  template <unsigned int NIn, unsigned int NOut>
  ControlSystem<NIn, NOut>::ControlSystem(ControlSystem&& control_system) noexcept
      : thread_running_(false), dirty_(false),
        rt_thread_(std::move(control_system.rt_thread_)),
        algorithms_(std::move(control_system.algorithms_)),
        ref_generator_(std::move(control_system.ref_generator_)),
        logger_(control_system.logger_)
  {
    thread_running_.store(control_system.thread_running_);
    dirty_.store(control_system.dirty_);

    logger_->write(LogLevel::Info, "Finished moving ControlSystem instance");
  }
  

  template <unsigned int NIn, unsigned int NOut>
  ControlSystem<NIn, NOut>::~ControlSystem()
  {
    // Wait for the real-time thread to finish executing, if at all
    try {
      if (rt_thread_.joinable()) {
        rt_thread_.join();
      }
    }
    catch (const std::system_error& e) {
      logger_->write(
          LogLevel::Error, "Exception raised when destroying ControlSystem "
              "object: ", e.what());
    }
  }


  template <unsigned int NIn, unsigned int NOut>
  template <typename Src, typename Snk>
  bool ControlSystem<NIn, NOut>::run(const unsigned int algorithm_id,
                                     const DataStore& parameters,
                                     Src& src, Snk& sink)
  {
    // Check if the thread is already running, and stop here if it is
    if (thread_running_) {
      logger_->write(
          LogLevel::Warning, "Trying to instruct controller when it's already "
              "running!");
      throw std::logic_error("Controller is already running!");
    }

    // Check to see if thread is joinable, and if so end it properly
    if (rt_thread_.joinable()) {
      try {
        rt_thread_.join();
      }
      catch (const std::system_error& e) {
        logger_->write(LogLevel::Error, "Exception raised when trying to join "
            "real-time thread: ", e.what());
        throw;
      }
    }

    wakeup_promise_ = std::promise<void>();

    const auto algorithm_index = algorithm_mapping_.at(algorithm_id);

    try {
      logger_->write(LogLevel::Info, "Launching controller thread...");
      rt_thread_ = std::thread(
          [this, algorithm_index, &parameters, &src, &sink]() {
        // Suspend thread until it's been made real-time (see below).
        wakeup_promise_.get_future().wait();
        // Flag the controller as busy
        thread_running_ = true;
        // Run the specified algorithm
        try {
          run_internal(algorithm_index, parameters, src, sink);
        }
        catch (const std::exception& e) {
          logger_->write(LogLevel::Error, "Unhandled exception in real-time "
                             "thread: ", e.what());
          sink(0u, static_cast<Vec<NOut>>(Vec<NOut>::Zero()));
          dirty_ = true;
        }
        thread_running_ = false;
      });
      logger_->write(LogLevel::Info, "Controller thread launched successfully!");
    }
    catch (const std::system_error& e) {
      logger_->write(LogLevel::Error, "Exception thrown when starting "
                         "real-time thread: ", e.what());
      throw;
    }

    logger_->write(LogLevel::Info, "Promoting controller thread to real-time "
        "priority...");
    // Set the priority of the new thread to maximum, making it real-time.
    struct sched_param param;
    param.sched_priority = 99;
    int result = pthread_setschedparam(rt_thread_.native_handle(),
                                       SCHED_FIFO, &param);

    if (result) {
      logger_->write(LogLevel::Warning, "Unable to make controller thread "
                         "real-time! (Error code ", result, ")");
    }
    else {
      logger_->write(LogLevel::Info, "Successfully made controller thread an "
          "RT thread.");
    }

    // Wake suspended real-time thread, now that it's real-time.
    wakeup_promise_.set_value();

    return true;
  }


  template <unsigned int NIn, unsigned int NOut>
  template<typename T, typename... Args>
  void ControlSystem<NIn, NOut>::add_algorithm(unsigned int id, Args&&... args)
  {
    if (is_busy()) {
      logger_->write(LogLevel::Warning,
                     "Cannot set algorithm: control system is busy!");
      return;
    }

    try {
      std::lock_guard<std::mutex> lock(global_mutex_);
      algorithms_.emplace_back(new T(std::forward<Args>(args)...));
      algorithm_mapping_[id] = algorithms_.size() - 1;
    }
    catch (const std::bad_alloc& e) {
      logger_->write(LogLevel::Error, "Unable to construct Algorithm instance: ",
                     e.what());
      throw;
    }
  }


  template <unsigned int NIn, unsigned int NOut>
  void ControlSystem<NIn, NOut>::add_algorithm(
      unsigned int id, std::unique_ptr<Algorithm<NIn, NOut>> algorithm)
  {
    if (is_busy()) {
      logger_->write(LogLevel::Warning,
                     "Cannot set algorithm: control system is busy");
      return;
    }

    try {
      std::lock_guard<std::mutex> lock(global_mutex_);
      algorithms_.push_back(std::move(algorithm));
      algorithm_mapping_[id] = algorithms_.size() - 1;
    }
    catch (const std::bad_alloc& e) {
      logger_->write(LogLevel::Error, "Unable to add Algorithm instance: ",
                     e.what());
      throw;
    }
  }


  template <unsigned int NIn, unsigned int NOut>
  template<typename T, typename... Args>
  void ControlSystem<NIn, NOut>::set_ref_generator(Args&&... args)
  {
    if (is_busy()) {
      logger_->write(
          LogLevel::Warning, "Cannot set reference generator: control system "
              "is busy!");
      return;
    }

    std::lock_guard<std::mutex> lock(global_mutex_);
    ref_generator_.reset(new T(std::forward<Args>(args)...));
  }

  
  template <unsigned int NIn, unsigned int NOut>
  void ControlSystem<NIn, NOut>::set_ref_generator(
      std::unique_ptr<RefGenerator<NIn>> ref_generator)
  {
    if (is_busy()) {
      logger_->write(LogLevel::Warning, "Cannot set reference generator: "
          "control system is busy");
      return;
    }

    std::lock_guard<std::mutex> lock(global_mutex_);
    ref_generator_ = std::move(ref_generator);
  }


  template <unsigned int NIn, unsigned int NOut>
  void ControlSystem<NIn, NOut>::erase_algorithm(const unsigned int id)
  {
    if (is_busy()) {
      logger_->write(LogLevel::Warning,
                     "Cannot erase algorithm: control system is busy");
    }
    if (algorithm_mapping_.count(id) == 0) {
      logger_->write(LogLevel::Warning, "ID supplied to erase_algorithm does "
          "not denote a valid algorithm.");
      return;
    }

    std::lock_guard<std::mutex> lock(global_mutex_);

    const auto idx = algorithm_mapping_.at(id);

    if (idx < algorithms_.size()) {
      algorithms_.erase(algorithms_.begin() + idx);

      for (auto& pair : algorithm_mapping_) {
        if (pair.second >= idx) {
          pair.second--;
        }
      }
    }
  }


  template <unsigned int NIn, unsigned int NOut>
  void ControlSystem<NIn, NOut>::stack_prefault()
  {
    // Pre-fault the stack by allocating a large array of bytes (5 MB)
    constexpr unsigned int array_size = 5 * 1024 * 1024;
    unsigned char big_array[array_size];
    std::fill(big_array, big_array + array_size, 0);
  }


  template <unsigned int NIn, unsigned int NOut>
  template <typename Src, typename Snk>
  void ControlSystem<NIn, NOut>::run_internal(
      const std::size_t algorithm_idx, const DataStore& parameters,
      Src& src, Snk& sink)
  {
    logger_->write(LogLevel::Info,
                   "Checking for algorithm and reference generator...");
    if (algorithm_idx >= algorithms_.size()) {
      logger_->write(LogLevel::Warning,
                     "Requested algorithm has not been set. Aborting run!");
      throw std::invalid_argument("Specified algorithm_idx is invalid!");
    }

    if (ref_generator_ == nullptr) {
      logger_->write(LogLevel::Warning,
                     "Reference generator has not been set. Aborting run!");
      throw std::logic_error("Reference generator has not been set!");
    }
    logger_->write(LogLevel::Info, "Algorithm and reference generator present!");

    // First: RT thread housekeeping -------------------------------------------

    logger_->write(LogLevel::Info, "Locking memory pages into RAM...");
    // Lock memory into RAM to prevent it being moved to disk
    int result = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (result == -1) {
      logger_->write(LogLevel::Warning, "Unable to lock memory pages!");
    }
    else {
      logger_->write(LogLevel::Info, "Successfully locked memory pages!");
    }

    // Prevent writing of variables while realtime code is running
    // (excepting joint angle array input_)
    std::lock_guard<std::mutex> global_lock(global_mutex_);

    // Pre-fault the stack to avoid page-faults during runs below.
    stack_prefault();

    logger_->write(LogLevel::Info, "Finished RT thread setup.");

    // Second: obtain and run algorithm components -----------------------------

    logger_->write(LogLevel::Info, "Grabbing and using algorithm...");

    auto& algorithm = *algorithms_[algorithm_idx];

    if (not thread_running_) {
      return;
    }

    algorithm.setup(parameters);

    // Prepare time-keeping objects
    std::vector<double> durations(ref_generator_->get_num_samples(), 0.0);
    const auto sleep_offset = compute_sleep_offset();
    const std::chrono::nanoseconds it_duration(1000000000l / frequency_);

    std::unique_lock<std::mutex> input_lock(input_mutex_, std::defer_lock);

    const auto start_time = std::chrono::steady_clock::now();

    for (unsigned int i = 0; i < ref_generator_->get_num_samples(); ++i) {
      const auto step_start_time = std::chrono::steady_clock::now();
      const std::chrono::nanoseconds drift = step_start_time - start_time;

      const InputType ref = ref_generator_->get_reference(i);
      const InputType input = src(i);
      const InputType correction = algorithm.get_ref_correction(i);
      const InputType error = ref + correction - input;

      algorithm.remember(error, i);

      const auto controller = algorithm.get_controller();
      const OutputType output = controller != nullptr ?
                                controller->compute_output(error) :
                                OutputType::Zero();

      sink(i, output);

      const auto end_time = std::chrono::steady_clock::now();
      std::chrono::nanoseconds comp_duration = end_time - step_start_time;

      durations[i] = comp_duration.count();
      logger_->write(LogLevel::Debug, "Real-time step took ",
                     comp_duration.count(), " nanoseconds");

      if (i > 0 and i % 10 == 0) {
        comp_duration += (drift - it_duration * i);
      }

      if (not thread_running_) {
        break;
      }

      if (it_duration > comp_duration + sleep_offset) {
        std::this_thread::sleep_for(it_duration - comp_duration - sleep_offset);
      }
    }

    algorithm.finish();

    logger_->write(LogLevel::Info, "Finished running algorithm!");

    // Compute some real-time thread time-keeping statistics

    const double min_duration =
        *std::min_element(durations.begin(), durations.end());
    const double mean_duration =
        std::accumulate(durations.begin(), durations.end(), 0.0)
            / durations.size();
    const double max_duration =
        *std::max_element(durations.begin(), durations.end());

    logger_->write(LogLevel::Info, "Real-time step duration stats:");
    logger_->write(LogLevel::Info, "Minimum duration = ", min_duration);
    logger_->write(LogLevel::Info, "Mean duration    = ", mean_duration);
    logger_->write(LogLevel::Info, "Maximum duration = ", max_duration);

    // Third: Restore memory state (unlocked) ----------------------------------

    logger_->write(LogLevel::Info, "Unlocking memory pages from RAM...");

    int unlock_result = munlockall();
    if (unlock_result == -1) {
      logger_->write(LogLevel::Warning, "Unable to unlock memory.");
    }
    else {
      logger_->write(LogLevel::Info, "Successfully unlocked memory.");
    }
  }
}

#endif //ASCENDANCY_CONTROLSYSTEM_HPP
