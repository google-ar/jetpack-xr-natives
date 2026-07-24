// Copyright 2026 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIRD_PARTY_SPLIT_ENGINE_CLIENT_TYPES_H_
#define THIRD_PARTY_SPLIT_ENGINE_CLIENT_TYPES_H_

#include <cstdint>
#include <memory>
#include <utility>

#include "absl/functional/any_invocable.h"

namespace split_engine {

// ResourceId is used to identify resources such as textures, materials, etc…
using ResourceId = std::uint64_t;

// EntityId is used to identify entities such as nodes, etc…
using EntityId = std::uint64_t;

// MessageGroupId is used to identify a group of messages.
using MessageGroupId = std::uint64_t;

// A wrapper around a type that will call a callback when it is destroyed.
template <typename T>
class WithDestructionCallback {
  using DestructionCallback = absl::AnyInvocable<void(const T&)>;

 public:
  WithDestructionCallback() = default;

  // Allow implicit construction from a value.
  WithDestructionCallback(T t) : t_(std::move(t)) {}

  explicit WithDestructionCallback(T t, DestructionCallback cb)
      : t_(std::move(t)), destruction_callback_(std::move(cb)) {}

  virtual ~WithDestructionCallback() {
    if (destruction_callback_) destruction_callback_(t_);
  }

  WithDestructionCallback(const WithDestructionCallback&) = delete;
  WithDestructionCallback& operator=(const WithDestructionCallback&) = delete;

  WithDestructionCallback(WithDestructionCallback&&) noexcept = default;
  WithDestructionCallback& operator=(WithDestructionCallback&& other) noexcept {
    if (this != &other) {
      if (destruction_callback_) destruction_callback_(t_);
      t_ = std::move(other.t_);
      destruction_callback_ = std::move(other.destruction_callback_);
    }
    return *this;
  }

  T* operator->() { return std::addressof(t_); }
  const T* operator->() const { return std::addressof(t_); }

  T& operator*() { return t_; }
  const T& operator*() const { return t_; }

  T& Get() { return t_; }
  const T& Get() const { return t_; }

 protected:
  T t_;
  DestructionCallback destruction_callback_ = nullptr;
};

}  // namespace split_engine

#endif  // THIRD_PARTY_SPLIT_ENGINE_CLIENT_TYPES_H_
