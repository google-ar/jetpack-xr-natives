/*
 * Copyright 2026 Google LLC
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
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_INPUT_EVENTS_POOL_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_INPUT_EVENTS_POOL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace imp {

enum class InputEventSource : uint8_t {
  kLocal = 0,
  kRemote = 1,
};

constexpr uint8_t kInputEventSourceCount = 2;

// Maintains events for a specific source.
template <typename T>
struct InputEventsBatch {
  InputEventSource source;
  std::vector<T> events;
};

// Maintains separate event queues for each InputEventSource (e.g., local vs.
// remote) to prevent interleaving, and provides methods to manage them.
template <typename T>
class InputEventsPool {
 public:
  using Source = InputEventSource;

  // Initializes the pool by setting up the source identifiers for each batch.
  InputEventsPool() {
    for (size_t i = 0; i < kInputEventSourceCount; ++i) {
      batches_[i].source = static_cast<InputEventSource>(i);
    }
  }

  // Adds an event to the queue associated with the specified source.
  void Add(T event, InputEventSource source) {
    batches_[static_cast<size_t>(source)].events.push_back(std::move(event));
  }

  // Returns a mutable reference to the raw events vector for a specific source.
  std::vector<T>& GetRawEvents(InputEventSource source) {
    return batches_[static_cast<size_t>(source)].events;
  }

  // Returns a const reference to the raw events vector for a specific source.
  const std::vector<T>& GetRawEvents(InputEventSource source) const {
    return batches_[static_cast<size_t>(source)].events;
  }

  // Provides direct mutable access to the batch for a specific source.
  InputEventsBatch<T>& operator[](InputEventSource source) {
    return batches_[static_cast<size_t>(source)];
  }

  // Provides direct const access to the batch for a specific source.
  const InputEventsBatch<T>& operator[](InputEventSource source) const {
    return batches_[static_cast<size_t>(source)];
  }

  // Checks if there are any events queued across all sources.
  bool HasEvents() const {
    for (const InputEventsBatch<T>& batch : batches_) {
      if (!batch.events.empty()) {
        return true;
      }
    }
    return false;
  }

  // Merges remote events into the local queue. If the local queue is not empty,
  // completely clear the remote queue. Otherwise, merge remote into local.
  void MergeSources() {
    std::vector<T>& local =
        batches_[static_cast<size_t>(InputEventSource::kLocal)].events;
    std::vector<T>& remote =
        batches_[static_cast<size_t>(InputEventSource::kRemote)].events;
    if (!local.empty()) {
      remote.clear();
      return;
    }

    if (!remote.empty()) {
      std::swap(local, remote);
    }
  }

  // Merges all sources into the local queue and returns the combined events.
  // Returns the merged events by value.
  std::vector<T> PopMergedEvents() {
    MergeSources();
    std::vector<T> result;
    std::swap(result,
              batches_[static_cast<size_t>(InputEventSource::kLocal)].events);
    return result;
  }

 private:
  std::array<InputEventsBatch<T>, kInputEventSourceCount> batches_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_INPUT_EVENTS_POOL_H_
