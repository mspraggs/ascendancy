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
 * Created by Matt Spraggs on 08/12/17
 */

#include "catch.hpp"

#include <EventQueue.hpp>


class Watcher
{
public:
  Watcher() : event_value_(0) {}

  void set_value(const int value) { event_value_ = value; }
  int get_value() const { return event_value_; }

private:
  int event_value_;
};


TEST_CASE("Test EventQueue")
{
  Watcher watcher;
  ascendancy::EventQueue<Watcher> queue;

  queue.register_link(
      watcher,
      [&watcher] (const ascendancy::DataStore& data) {
        watcher.set_value(data.get<int>());
      }
  );

  SECTION("Test raise_event sync")
  {
    queue.raise_event(watcher, 3);
    queue.consume_event();

    REQUIRE(watcher.get_value() == 3);
  }

  SECTION("Test raise_event async")
  {
    queue.start();
    queue.raise_event(watcher, 5);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    REQUIRE(watcher.get_value() == 5);
  }

  SECTION("Test unregister_link")
  {
    queue.unregister_link(watcher);

    queue.raise_event(watcher, 3);
    queue.consume_event();
  }

  SECTION("Test start/stop")
  {
    queue.start();
    queue.raise_event(watcher, 6);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    queue.stop();

    queue.raise_event(watcher, 2);

    REQUIRE(watcher.get_value() == 6);
  }
}