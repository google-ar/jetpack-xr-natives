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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_CONCEPTS_SHARED_MEMORY_TRANSPORT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_CONCEPTS_SHARED_MEMORY_TRANSPORT_H_

#if __cpp_concepts

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/common/invocable.h"
#include "core/split_engine/android/buffer_handle_factory.h"
#include "core/split_engine/transport/concepts/underlying_transport.h"

namespace imp::split_engine {
template <typename T>
concept SharedMemoryTransport =
    UnderlyingTransport<T> && requires(T& transport) {
      {
        transport.ProcessRegion(
            std::declval<BufferHandleFactory::BufferHandle>(), 0, 0)
      } -> std::same_as<absl::Status>;

      {
        transport.SendRequest(
            std::declval<absl::Span<const uint8_t>>(),
            std::declval<imp::Invocable<void(absl::Span<const uint8_t>)>>())
      } -> std::same_as<absl::Status>;

      {
        transport.RegisterBuffer(std::declval<int>(), std::declval<size_t>())
      } -> std::same_as<
          absl::StatusOr<std::unique_ptr<BufferHandleFactory::BufferHandle>>>;
    };
}  // namespace imp::split_engine

#endif  // __cpp_concepts

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TRANSPORT_CONCEPTS_SHARED_MEMORY_TRANSPORT_H_
