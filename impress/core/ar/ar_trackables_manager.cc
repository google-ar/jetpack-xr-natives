// Copyright 2024 Google LLC
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

#include "core/ar/ar_trackables_manager.h"

#include "core/ar/ar_trackable_handle.h"

namespace imp {
namespace ar {
namespace {
// Disables ArTrackableHandles when the TrackablesManager is destroyed.
struct Invalidator {
  explicit Invalidator(ArTrackablesManager* trackables_manager)
      : trackables_manager(trackables_manager) {}
  template <typename T>
  constexpr void operator()(T&&) {
    using TrackableType = typename std::remove_reference<T>::type::value_type;
    trackables_manager->DisableHandleType<ArTrackableHandle<TrackableType>>();
  }
  ArTrackablesManager* trackables_manager;
};
// Enables ArTrackableHandles when the TrackablesManager is constructed.
struct Validator {
  explicit Validator(ArTrackablesManager* trackables_manager)
      : trackables_manager(trackables_manager) {}
  template <typename T>
  constexpr void operator()(T&&) {
    using TrackableType = typename std::remove_reference<T>::type::value_type;
    trackables_manager->EnableHandleType<ArTrackableHandle<TrackableType>>();
  }
  ArTrackablesManager* trackables_manager;
};
}  // namespace

ArTrackablesManager::ArTrackablesManager() {
  Validator functor(this);
  TrackableTuple just_types;
  imp::ForEachTupleElement(just_types, functor);
}

ArTrackablesManager::~ArTrackablesManager() {
  Invalidator functor(this);
  TrackableTuple just_types;
  imp::ForEachTupleElement(just_types, functor);
}

}  // namespace ar
}  // namespace imp
