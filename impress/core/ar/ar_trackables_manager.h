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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLES_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLES_MANAGER_H_

#include <tuple>

#include "core/ar/ar_frame.h"
#include "core/ar/ar_plane.h"
#include "core/ar/ar_trackable.h"
#include "core/common/robin_map.h"
#include "core/common/tuple_helpers.h"

namespace imp {
namespace ar {

template <typename T>
using TrackableMap = RobinMap<ArTrackableId, T>;

// Manages a map of trackabes, provides a single reference to data required for
// ArTrackableHandles.
class ArTrackablesManager {
 public:
  ArTrackablesManager();
  ~ArTrackablesManager();
  // Retrieves a trackable from the map or nullptr.
  template <typename T>
  T* GetTrackable(ArTrackableId id) const;
  template <typename T>
  T* GetTrackable(ArTrackableId id);

  // Adds a trackable into the map at slot 'id'.
  template <typename T>
  void AddOrUpdateTrackable(const T& trackable, ArTrackableId id);

  // Erases a trackable from the map.
  template <typename T>
  void ClearTrackable(ArTrackableId id);

  
  // Returns the underlying tuple of trackable maps.
  TrackableMapTuple& GetTuple() { return managed_trackable_map_; }

  // Disables every handle of HandleType.
  template <typename HandleType>
  void DisableHandleType();

  // Enable every handle of HandleType.
  template <typename HandleType>
  void EnableHandleType();

 private:
  // Helper functor for finding a trackable by Id and representing as a base
  // trackable.
  struct GetAsBaseTrackable {
    GetAsBaseTrackable(const ArTrackablesManager* trackable_manager,
                       ArTrackableId find_id)
        : trackable_manager(trackable_manager), find_id(find_id) {}
    template <typename TrackableList>
    constexpr void operator()(TrackableList) {
      using TrackableType =
          typename std::remove_reference<TrackableList>::type::value_type;

      auto& map = std::get<TrackableMap<TrackableType>>(
          trackable_manager->managed_trackable_map_);
      auto iter = map.find(find_id);
      if (iter != map.end()) {
        found_trackable = &iter.value();
      }
    }
    const ArTrackable* found_trackable = nullptr;
    const ArTrackablesManager* trackable_manager;
    ArTrackableId find_id;
  };

  TrackableMapTuple managed_trackable_map_;
};

template <typename T>
T* ArTrackablesManager::GetTrackable(ArTrackableId id) const {
  if constexpr (std::is_same_v<T, ar::ArTrackable>) {
    GetAsBaseTrackable get_base_trackable(this, id);
    TrackableTuple just_types;
    imp::ForEachTupleElement(just_types, get_base_trackable);
    return const_cast<T*>(get_base_trackable.found_trackable);
  } else {
    auto& map = std::get<TrackableMap<T>>(managed_trackable_map_);
    auto iter = map.find(id);
    if (iter != map.end()) {
      return const_cast<T*>(&iter.value());
    }
  }

  return nullptr;
}

template <typename T>
T* ArTrackablesManager::GetTrackable(ArTrackableId id) {
  return const_cast<T*>(
      static_cast<const ArTrackablesManager*>(this)->GetTrackable<T>(id));
}

template <typename T>
void ArTrackablesManager::AddOrUpdateTrackable(const T& trackable,
                                               ArTrackableId id) {
  auto& map = std::get<TrackableMap<T>>(managed_trackable_map_);
  map[id] = trackable;
}

template <typename T>
void ArTrackablesManager::ClearTrackable(ArTrackableId id) {
  auto& map = std::get<TrackableMap<T>>(managed_trackable_map_);
  map.erase(id);
}

template <typename HandleType>
void ArTrackablesManager::DisableHandleType() {
  HandleType::session_is_valid_ = false;
}

template <typename HandleType>
void ArTrackablesManager::EnableHandleType() {
  HandleType::session_is_valid_ = true;
}

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLES_MANAGER_H_
