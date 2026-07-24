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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMMON_NOTIFYING_PROPERTY_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMMON_NOTIFYING_PROPERTY_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/container/inlined_vector.h"
#include "core/common/invocable.h"
#include "core/common/ref_counter.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

// Forward declarations
template <typename T>
class PropertyBinding;

// Defines how the callback is connected to the property at bind construction.
enum class BindingMode {
  // Only bind the callback to the property.
  kBind,
  // Bind the callback to the property and fire it immediately.
  kBindAndFire,
};

// Type-agnostic structures designed to minimize binary size.
// Without these the compiler would generate a lot of duplicate code.
struct SubscriptionBlockBase {
  virtual ~SubscriptionBlockBase() = default;
  bool active = true;
};

struct StateBase {
  // The number of subscriptions to reserve space for in the inlined vector.
  // The value was chosen because on a 64-bit system, 2 inlined pointers can
  // be stored for the price of 1. Raising this value will increase the size of
  // this struct.
  static constexpr size_t kVectorReserve = 2;

  virtual ~StateBase() = default;

  void Cleanup() {
    for (SubscriptionBlockBase* sub : subscriptions) {
      sub->active = false;
      ClearCallback(sub);
    }
    subscriptions.clear();
    dead_subscriptions.clear();
  }

  virtual void ClearCallback(SubscriptionBlockBase* sub) = 0;

  absl::InlinedVector<SubscriptionBlockBase*, kVectorReserve> subscriptions;
  bool property_alive = true;
  uint32_t notification_depth = 0;
  imp::RefCounter lifetime_tracker;
  absl::InlinedVector<std::unique_ptr<SubscriptionBlockBase>, kVectorReserve>
      dead_subscriptions;
};

// Base class containing type-agnostic state/logic for NotifyingProperty.
class NotifyingPropertyBase {
 public:
  // NotifyingPropertyBase is move-only.
  NotifyingPropertyBase(const NotifyingPropertyBase&) = delete;
  NotifyingPropertyBase& operator=(const NotifyingPropertyBase&) = delete;

 protected:
  NotifyingPropertyBase() = default;
  explicit NotifyingPropertyBase(std::unique_ptr<StateBase> state)
      : state_(std::move(state)) {}

  ~NotifyingPropertyBase() { Cleanup(); }

  NotifyingPropertyBase(NotifyingPropertyBase&& other) noexcept = default;
  NotifyingPropertyBase& operator=(NotifyingPropertyBase&& other) noexcept {
    if (this == &other) return *this;

    Cleanup();
    state_ = std::move(other.state_);

    return *this;
  }

  void Cleanup() noexcept {
    if (!state_) return;

    state_->property_alive = false;

    // If a callback deletes or reassigns the property, release unique_ptr.
    // Then at the end of Set() we delete the state rather than doing it here.
    if (state_->notification_depth != 0) {
      state_.release();
      return;
    }

    state_->Cleanup();
    state_.reset();
  }

  std::unique_ptr<StateBase> state_;
};

// Base class containing type-agnostic state/logic for PropertyBinding.
class PropertyBindingBase {
 public:
  // The number of subscriptions to reserve space for in the inlined vector.
  static constexpr size_t kVectorReserve = 2;

  PropertyBindingBase() = default;
  ~PropertyBindingBase() { Unbind(); }

  // PropertyBindingBase is move-only, ensuring single ownership of the
  // subscription.
  PropertyBindingBase(const PropertyBindingBase&) = delete;
  PropertyBindingBase& operator=(const PropertyBindingBase&) = delete;

  PropertyBindingBase(PropertyBindingBase&& other) noexcept {
    *this = std::move(other);
  }

  PropertyBindingBase& operator=(PropertyBindingBase&& other) noexcept {
    if (this != &other) {
      Unbind();
      state_ = other.state_;
      sub_block_ = std::move(other.sub_block_);
      ref_token_ = std::move(other.ref_token_);
      other.state_ = nullptr;
    }
    return *this;
  }

  // Returns true if the binding is currently active.
  bool IsBound() const {
    return state_ && !ref_token_.IsCounterDestroyed() && sub_block_ &&
           sub_block_->active;
  }

 protected:
  // Manually disconnects from the notifying property.
  void Unbind() {
    if (!state_ || !sub_block_) return;

    sub_block_->active = false;

    // Needs to occur regardless of how we got here.
    const absl::Cleanup _ = [this] {
      state_ = nullptr;
      sub_block_.reset();
      ref_token_ = {};
    };

    // If the property has already been destroyed, there's nothing left to do.
    if (ref_token_.IsCounterDestroyed()) return;

    // If we're in a callback, defer the actual subscription removal.
    if (state_->notification_depth > 0) {
      state_->dead_subscriptions.push_back(std::move(sub_block_));
      return;
    }

    // Actual subscription removal.
    absl::InlinedVector<SubscriptionBlockBase*, kVectorReserve>& subs =
        state_->subscriptions;

    for (auto it = subs.begin(); it != subs.end(); ++it) {
      if (*it != sub_block_.get()) continue;
      subs.erase(it);
      break;
    }
  }

  StateBase* state_ = nullptr;
  std::unique_ptr<SubscriptionBlockBase> sub_block_;
  imp::RefCounter::Ref ref_token_;
};

// NotifyingProperty holds a value of type T and registers observers that are
// notified when the value changes. T must support operator!= and operator==.
// This class is not thread-safe.
//
// NOTE: This class has heap allocation and performance overhead:
// - It allocates a control block (`State`) on the heap.
// - Each subscription holds a separate heap-allocated `SubscriptionBlock`.
// - Set() iterates over active subscriptions in-place using depth-tracking
//   and deferred mutation to handle re-entrancy safely.
// As such, it should be used selectively (e.g., for high-level UI/editor state
// rather than per-frame hot-path component data).
template <typename T>
class NotifyingProperty : public NotifyingPropertyBase {
 public:
  NotifyingProperty() : NotifyingPropertyBase(std::make_unique<State>()) {}
  explicit NotifyingProperty(T value)
      : NotifyingPropertyBase(std::make_unique<State>(std::move(value))) {}

  ~NotifyingProperty() = default;

  // NotifyingProperty is move-constructible and move-assignable,
  // but not copyable.
  NotifyingProperty(const NotifyingProperty&) = delete;
  NotifyingProperty& operator=(const NotifyingProperty&) = delete;

  NotifyingProperty(NotifyingProperty&& other) noexcept = default;
  NotifyingProperty& operator=(NotifyingProperty&& other) noexcept = default;

  // Set the value of the property and notify all active bindings if it changed.
  void Set(T new_value) {
    State* local_state = GetTypedState();
    if (local_state->value == new_value) return;

    // Use raw pointer for loop/cleanup because state_ unique_ptr might be
    // released by the destructor during callbacks.
    const T old_value = std::move(local_state->value);
    local_state->value = std::move(new_value);

    local_state->notification_depth++;

    // Fire off all active bindings.
    const size_t original_size = local_state->subscriptions.size();
    for (size_t i = 0; i < original_size; ++i) {
      if (!local_state->property_alive) break;

      SubscriptionBlockBase* base_sub = local_state->subscriptions[i];
      if (!base_sub->active) continue;

      auto* sub = static_cast<SubscriptionBlock*>(base_sub);

      if (!sub->callback) continue;

      // Only run the callback if the property is still alive, the subscription
      // is active, and the callback is valid.
      sub->callback(local_state->value, old_value);
    }

    local_state->notification_depth--;

    // We're in a callback, so we can't clean up the state yet.
    if (local_state->notification_depth != 0) return;

    // Remove inactive subscriptions from the list.
    absl::InlinedVector<SubscriptionBlockBase*, StateBase::kVectorReserve>&
        subs = local_state->subscriptions;
    const auto inactive = [](const SubscriptionBlockBase* sub) {
      return !sub->active;
    };
    subs.erase(std::remove_if(subs.begin(), subs.end(), inactive), subs.end());

    // Clean up any subscriptions that were destroyed in the callback.
    local_state->dead_subscriptions.clear();

    if (!local_state->property_alive) {
      // If the property was destroyed in a callback, we clean up the state.
      // Check the base class Cleanup() to see why this raw delete is okay.
      local_state->Cleanup();
      delete local_state;
    }
  }

  // Retrieve the current value.
  const T& Get() const { return GetTypedState()->value; }

  // Dereference operators.
  const T& operator*() const { return Get(); }
  const T* operator->() const { return &GetTypedState()->value; }

  // Assignment operator.
  NotifyingProperty& operator=(T new_value) {
    Set(std::move(new_value));
    return *this;
  }

 private:
  friend class PropertyBinding<T>;

  struct SubscriptionBlock : public SubscriptionBlockBase {
    imp::Invocable<void(const T&, const T&)> callback;
  };

  struct State : public StateBase {
    State() = default;
    explicit State(T val) : value(std::move(val)) {}

    void ClearCallback(SubscriptionBlockBase* sub) override {
      static_cast<SubscriptionBlock*>(sub)->callback = {};
    }

    T value;
  };

  State* GetTypedState() const { return static_cast<State*>(state_.get()); }
};

// PropertyBinding automates subscribing to and unsubscribing from a
// NotifyingProperty. When constructed or assigned, it binds a callback
// to the property. Upon destruction or reassignment, it automatically unbinds.
// This class is not thread-safe.
template <typename T>
class PropertyBinding : public PropertyBindingBase {
 public:
  using Callback = imp::Invocable<void(const T& new_value, const T& old_value)>;
  using OnBindCallback = imp::Invocable<void(const T& current_value)>;
  using SubscriptionBlock = typename NotifyingProperty<T>::SubscriptionBlock;

  PropertyBinding() = default;

  // Constructor with onChanged and 1-parameter onBind callbacks.
  // For when onBind needs different logic than the onChanged callback.
  PropertyBinding(NotifyingProperty<T>& property, Callback callback,
                  OnBindCallback on_bind) {
    Bind(property, std::move(callback), std::move(on_bind));
  }

  // Constructor that takes a BindingMode enum (defaulting to kBind) to
  // determine whether the callback should be executed immediately.
  PropertyBinding(NotifyingProperty<T>& property, Callback callback,
                  BindingMode mode = BindingMode::kBind) {
    Bind(property, std::move(callback), mode);
  }

  ~PropertyBinding() = default;

  // PropertyBinding is move-only, ensuring single ownership of the
  // subscription.
  PropertyBinding(const PropertyBinding&) = delete;
  PropertyBinding& operator=(const PropertyBinding&) = delete;

  PropertyBinding(PropertyBinding&& other) noexcept = default;
  PropertyBinding& operator=(PropertyBinding&& other) noexcept = default;

 private:
  // Connects to a notifying property with a 1-parameter onBind.
  void Bind(NotifyingProperty<T>& property, Callback callback,
            OnBindCallback on_bind) {
    BindImpl(property, std::move(callback));

    if (!on_bind) return;

    on_bind(static_cast<typename NotifyingProperty<T>::State*>(state_)->value);
  }

  // Connects to a notifying property, executing the callback immediately if
  // mode is kBindAndFire (default is kBind).
  void Bind(NotifyingProperty<T>& property, Callback callback,
            BindingMode mode = BindingMode::kBind) {
    BindImpl(property, std::move(callback));

    if (mode != BindingMode::kBindAndFire) return;

    const auto* sub = static_cast<SubscriptionBlock*>(sub_block_.get());
    if (!sub->callback) return;

    const T& value =
        static_cast<typename NotifyingProperty<T>::State*>(state_)->value;

    sub->callback(value, value);
  }

  void BindImpl(NotifyingProperty<T>& property, Callback callback) {
    Unbind();

    state_ = property.state_.get();
    ref_token_ = state_->lifetime_tracker.Retain();

    auto sub = std::make_unique<SubscriptionBlock>();
    sub->callback = std::move(callback);
    sub->active = true;

    state_->subscriptions.push_back(sub.get());
    sub_block_ = std::move(sub);
  }
};

// Bridge function to automatically dispatch events on an imp::Dispatcher
// when a NotifyingProperty's value changes.
// EventType must be constructible from 'const T&' or 'T'.
template <typename EventType, typename T>
PropertyBinding<T> BindPropertyToDispatcher(NotifyingProperty<T>& property,
                                            imp::Dispatcher& dispatcher,
                                            imp::NodeHandle node = {},
                                            bool dispatch_immediately = true) {
  // Callback triggers when property changes
  typename PropertyBinding<T>::Callback callback =
      [&dispatcher, node](const T& value, const T& /*old_value*/) {
        dispatcher.Send(node, EventType(value));
      };

  typename PropertyBinding<T>::OnBindCallback on_bind;
  if (dispatch_immediately) {
    on_bind = [&dispatcher, node](const T& value) {
      dispatcher.Send(node, EventType(value));
    };
  }

  return PropertyBinding<T>(property, std::move(callback), std::move(on_bind));
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMMON_NOTIFYING_PROPERTY_H_
