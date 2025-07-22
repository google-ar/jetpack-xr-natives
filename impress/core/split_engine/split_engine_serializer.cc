/*
 * Copyright 2025 Google LLC
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

#include "core/split_engine/split_engine_serializer.h"

#include <cstdint>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/types/span.h"

namespace imp::split_engine {

class SplitEngineResourceIdManager {
 public:
  static SplitEngineResourceIdManager& GetInstance() {
    static SplitEngineResourceIdManager instance;
    return instance;
  }

  std::uint64_t GetId(const void* ptr) {
    const auto it = ptr_to_id_.find(ptr);
    if (it != ptr_to_id_.end()) {
      return it->second;
    }
    const uint64_t id = ++next_id_;
    ptr_to_id_[ptr] = id;
    id_to_ptr_[id] = ptr;
    return id;
  }

  void RemoveIds(absl::Span<const uint64_t> ids) {
    for (const auto& id : ids) {
      RemoveId(id);
    }
  }

  void RemoveId(uint64_t id) {
    const auto it = id_to_ptr_.find(id);
    
    RemovePtr(it->second);
    id_to_ptr_.erase(it);
  }

 private:
  SplitEngineResourceIdManager() = default;
  SplitEngineResourceIdManager(const SplitEngineResourceIdManager&) = delete;
  SplitEngineResourceIdManager& operator=(const SplitEngineResourceIdManager&) =
      delete;
  SplitEngineResourceIdManager(SplitEngineResourceIdManager&&) = delete;
  SplitEngineResourceIdManager& operator=(SplitEngineResourceIdManager&&) =
      delete;

  void RemovePtr(const void* ptr) {
    const auto it = ptr_to_id_.find(ptr);
    
    ptr_to_id_.erase(it);
  }

  // Unique ID generator
  uint64_t next_id_ = 0;
  // Maps a pointer to its ID
  absl::flat_hash_map<const void*, uint64_t> ptr_to_id_;
  // Maps an ID to its pointer
  absl::flat_hash_map<uint64_t, const void*> id_to_ptr_;
};

uint64_t SplitEngineSerializer::GetId(const void* ptr) {
  return SplitEngineResourceIdManager::GetInstance().GetId(ptr);
}

void SplitEngineSerializer::RemoveId(uint64_t id) {
  SplitEngineResourceIdManager::GetInstance().RemoveId(id);
}

}  // namespace imp::split_engine
