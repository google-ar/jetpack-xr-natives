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

#ifndef THIRD_PARTY_SPLIT_ENGINE_SPLIT_ENGINE_SUBSPACE_MANAGER_IMPL_H_
#define THIRD_PARTY_SPLIT_ENGINE_SPLIT_ENGINE_SUBSPACE_MANAGER_IMPL_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "absl/status/status.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/async/executor.h"
#include "core/common/rememberer.h"
#include "core/math/mat.h"
#include "core/view/base_view.h"
#include "split_engine/input/split_engine_input_event.h"
#include "split_engine/split_engine_subspace_manager.h"
#include "split_engine/subspace_root.h"

namespace android_xr {

class HashEntity {
 public:
  size_t operator()(utils::Entity entity) const { return entity.getId(); }
};

// TODO: Split the BaseRenderableManager code to a separate class.
// TODO: Add mutex guard when accessing variables that are shared
// among threads.
class SplitEngineSubspaceManagerImpl : public SplitEngineSubspaceManager,
                                       public imp::Rememberer {
 public:
  explicit SplitEngineSubspaceManagerImpl(imp::BaseView& view);

  void DestroyAllSubspaces() override {
    for (auto& [subspace_id, subspace_root] : subspace_map_) {
      DestroySubspace(subspace_id);
    }
  }

  uint32_t GetNextSubspaceId() override {
    return subspace_id_generator_.GetNextId();
  };

  void RegisterSubspace(uint32_t subspace_id,
                        uint32_t existing_root_entity_id) override;

  void CreateSubspace(uint32_t subspace_id,
                              std::string app_name) override;

  void DestroySubspace(uint32_t subspace_id) override;

  void ForwardSubspaceTransform(
      uint32_t subspace_id, const imp::mat4f& subspace_transform) override;

  // Call only from frame thread.
  absl::Status ForwardInputEvent(
      uint32_t subspace_id,
      android_xr::SplitEngineInputEvent& input_event) override;

  void UpdateSubspaceAnchor(
      uint32_t subspace_id, SubspaceRoot::AnchorType anchor_type) override;

 private:
  struct SubspaceIdGenerator {
    uint32_t GetNextId() { return next_id++; }
    uint32_t next_id = 1;
  } subspace_id_generator_;

  std::unordered_map<uint32_t, SubspaceRoot> subspace_map_;

  imp::BaseView& view_;
  imp::Executor* foreground_executor_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_SPLIT_ENGINE_SUBSPACE_MANAGER_IMPL_H_
