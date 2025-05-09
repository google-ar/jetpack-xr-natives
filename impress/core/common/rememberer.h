/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_REMEMBERER_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_REMEMBERER_H_

#include <cstdint>
#include <memory>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/synchronization/mutex.h"
#include "core/common/holdable.h"
#include "core/common/invocable.h"

namespace imp {

// Helper for determining if type T is able to remember objects.
//
// This is true if T is a pointer-like object that contains a public method with
// the following signature:
//
// Invocable<void()> Remember(Holdable);
//
// This is different from just checking if a type inherits from Rememberer
// because in some cases a type wraps a Rememberer instead of inheriting, and
// then forwards calls to a Rememberer by implementing its own Remember method.
template <typename T, typename = int>
struct CanTypeRemember : std::false_type {};
template <typename T>
struct CanTypeRemember<
    T, decltype(std::declval<T>()->Remember(std::declval<Holdable>()), 0)>
    : std::is_same<decltype(std::declval<T>()->Remember(
                       std::declval<Holdable>())),
                   Invocable<void()>> {};

// Utility class that can 'remember' objects to tie the object lifetime of the
// remembered object to the Rememberer.
//
// The remembered objects passed in must be of type 'Holdable' which is a
// move-only type-erased wrapper for arbitrary objects. Holdable can store value
// types, reference-counted types, and move only types. It will not deallocate
// raw pointers.
//
// For example, Futures, Resources, or ScopedConnections can all be remembered
// by a Rememberer as a Holdable, preventing them from being destroyed during
// the lifetime of the Rememberer.
class Rememberer {
 public:
  Rememberer();
  ~Rememberer();

  Rememberer(const Rememberer&) = delete;
  Rememberer& operator=(const Rememberer& rhs) = delete;
  Rememberer(Rememberer&& rhs) = default;
  Rememberer& operator=(Rememberer&& rhs) = default;

  // Remembers the holdable and returns a function to forget it.
  //
  // Remember can be called from any thread.
  //
  // The returned functor can be called from any thread to tell the
  // rememberer that it can stop holding onto the remembered object. The
  // remembered object is guaranteed to be forgotten on the foreground executor
  // thread. If the functor is called from a background thread, a task will be
  // scheduled to the foreground executor thread to forget the object. It is
  // also safe to call the forgetter after the Rememberer is destroyed.
  //
  //
  // If the Rememberer is being destructed, this function returns an empty
  // function and does not remember the Holdable.
  Invocable<void()> Remember(Holdable holdable);

  // Returns true if the rememberer is currently remembering any objects.
  bool HasRemembered() const;

  // Clears all kept futures so they can be cancelled. If this is not called,
  // then the kept futures will be cleared automatically upon destruction.
  //
  // However, it is very useful to call this explicitly in a destructor or
  // cleanup function to ensure that the kept futures are cleared prior to other
  // cleanup work in case the futures depend on memory that would be destroyed
  // before the destructor for this class is run.
  void ClearRemembered();

 private:
  using Id = uint64_t;
  // We use a Holdable to store type erased objects.
  using RememberedObjectsMap = absl::flat_hash_map<Id, Holdable>;

  struct RemembererInfo {
    absl::Mutex mu;

    Id next_id ABSL_GUARDED_BY(mu) = 0;

    RememberedObjectsMap remembered_objects_map ABSL_GUARDED_BY(mu);

    // Used to block Holdables from being remembered during Rememberer
    // destruction.
    bool can_remember_object ABSL_GUARDED_BY(mu) = true;
  };

  void ClearRememberedInternal(bool is_destroying_rememberer);

  Invocable<void()> GetForgetFunction(const Id& id);

  // Store information about remembered objects in a unique_ptr so that a
  // Rememberer can be std::moved without invalidating the forget functions.
  std::shared_ptr<RemembererInfo> info_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_KEEPALIVE_HELPERS_H_
