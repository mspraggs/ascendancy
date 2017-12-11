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
 * Created by Matt Spraggs on 11/10/17
 */

#ifndef ASCENDANCY_EVENTQUEUE_HPP
#define ASCENDANCY_EVENTQUEUE_HPP

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <condition_variable>

#include  "DataStore.hpp"


namespace ascendancy
{
  namespace detail
  {
    template <typename T>
    void args_to_datastore(DataStore& store, T&& arg)
    {
      store.set(arg);
    }

    template <typename T, typename... Ts>
    void args_to_datastore(DataStore& store, T&& arg0, Ts&&... args)
    {
      store.set(arg0);
      args_to_datastore(store, std::forward<Ts>(args)...);
    }
  }


  template <typename Base>
  class EventQueueBase
  {
    // Looks like a pointless base class, but useful for mocking in tests
  private:
    using Slot = std::function<void(const DataStore&)>;

  protected:
    virtual void raise_event_impl(const Base& subject, DataStore data) {}

  public:
    virtual ~EventQueueBase() = default;

    virtual void start() {}
    virtual void stop() {}

    virtual void register_link(const Base& subject, Slot slot) {}
    virtual void unregister_link(const Base& subject) {}

    template <typename... Args>
    void raise_event(const Base& subject, Args&&... args)
    {
      DataStore data;
      detail::args_to_datastore(data, std::forward<Args>(args)...);
      raise_event_impl(subject, std::move(data));
    }

    virtual void consume_event() {}
  };


  template <typename Base>
  class EventQueue : public EventQueueBase<Base>
  {
  private:
    using Slot = std::function<void(const DataStore&)>;

  public:
    EventQueue()
        : running_(false), queue_head_(0), queue_tail_(0), queue_size_(0)
    {
    }

    ~EventQueue();

    void start() override;
    void stop() override;

    void register_link(const Base& subject, Slot slot) override;
    void unregister_link(const Base& subject) override;

    void raise_event_impl(const Base& subject, DataStore args) override;

    void consume_event() override;

  private:
    void watch_queue();
    void consume_event(std::unique_lock<std::mutex>&& queue_lock);

    static constexpr unsigned int max_queue_size_ = 4096;

    std::atomic_bool running_;
    unsigned int queue_head_, queue_tail_;
    std::atomic_uint queue_size_;

    std::mutex queue_mutex_, slots_mutex_;
    std::condition_variable queue_condition_;
    std::thread queue_watcher_;

    std::array<std::tuple<const Base*, DataStore>, max_queue_size_> queue_;

    std::unordered_map<const Base*, std::size_t> slot_map_;
    std::vector<std::vector<Slot>> slots_;
  };


  template<typename Base>
  EventQueue<Base>::~EventQueue()
  {
    stop();
  }


  template<typename Base>
  void EventQueue<Base>::start()
  {
    if (running_) {
      return;
    }

    queue_watcher_ = std::thread([this] () { watch_queue(); });
  }


  template<typename Base>
  void EventQueue<Base>::stop()
  {
    if (not running_) {
      return;
    }

    queue_size_ = 1;
    running_ = false;
    queue_condition_.notify_one();

    if (queue_watcher_.joinable()) {
      queue_watcher_.join();
    }
  }


  template<typename Base>
  void EventQueue<Base>::register_link(const Base& subject,
                                       EventQueue::Slot slot)
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);

    const bool slot_exists = slot_map_.count(&subject) > 0;
    const std::size_t idx = slot_exists ? slot_map_[&subject] : slots_.size();

    if (not slot_exists) {
      slots_.emplace_back();
      slot_map_[&subject] = idx;
    }
    slots_[idx].push_back(std::move(slot));
  }


  template<typename Base>
  void EventQueue<Base>::unregister_link(const Base& subject)
  {
    std::lock_guard<std::mutex> lock(slots_mutex_);

    const bool slot_exists = slot_map_.count(&subject) > 0;
    if (not slot_exists) {
      return;
    }

    const std::size_t idx = slot_map_[&subject];
    slot_map_.erase(&subject);
    slots_.erase(slots_.begin() + idx);
  }


  template<typename Base>
  void EventQueue<Base>::raise_event_impl(const Base& subject, DataStore data)
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (queue_size_ >= max_queue_size_) {
      // TODO: Handling of queue overflow
      return;
    }

    queue_[queue_tail_] = std::make_tuple(&subject, std::move(data));

    queue_size_++;
    queue_tail_ = (queue_tail_ + 1) % max_queue_size_;

    queue_condition_.notify_one();
  }


  template<typename Base>
  void EventQueue<Base>::consume_event()
  {
    if (running_ or queue_size_ == 0) {
      return;
    }

    std::unique_lock<std::mutex> queue_lock(queue_mutex_);
    consume_event(std::move(queue_lock));
  }


  template<typename Base>
  void EventQueue<Base>::watch_queue()
  {
    running_ = true;
    while (running_ or queue_size_ > 0) {

      std::unique_lock<std::mutex> queue_lock(queue_mutex_);
      queue_condition_.wait(queue_lock, [this] () { return queue_size_ > 0; });

      if (not running_) {
        return;
      }

      consume_event(std::move(queue_lock));
    }
  }


  template <typename Base>
  void EventQueue<Base>::consume_event(std::unique_lock<std::mutex>&& queue_lock)
  {
    const auto queue_item = std::move(queue_[queue_head_]);
    queue_size_--;
    queue_head_ = (queue_head_ + 1) % max_queue_size_;

    queue_lock.unlock();

    const auto subject = std::get<0>(queue_item);

    std::lock_guard<std::mutex> slots_lock(slots_mutex_);

    if (slot_map_.count(subject) == 0) {
      return;
    }

    for (auto& slot : slots_[slot_map_[subject]]) {
      slot(std::get<1>(queue_item));
    }
  }
}

#endif //ASCENDANCY_EVENTQUEUE_HPP
