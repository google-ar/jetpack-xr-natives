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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_TEMPORARY_EDITOR_METADATA_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_TEMPORARY_EDITOR_METADATA_H_
#include <cstdint>

#include "core/editor/components/temporary_editor_metadata_state.proto.imp.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"

namespace imp::editor {

// Serialized component used to assign additional tracking data to nodes, that
// are destroyed and recreated during editor mode switches.
class TemporaryEditorMetadata : public Component {
 public:
  void SetId(int64_t id) { state_.editor_node_id = id; }
  int64_t GetId() const { return state_.editor_node_id; }

 private:
  TemporaryEditorMetadataState state_;

 public:
  static constexpr bool kExcludeFromEditor = true;
  using IsfInfo = IsfInfo<&TemporaryEditorMetadata::state_>;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_COMPONENTS_TEMPORARY_EDITOR_METADATA_H_
