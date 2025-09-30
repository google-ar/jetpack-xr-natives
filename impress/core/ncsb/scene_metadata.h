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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_SCENE_METADATA_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_SCENE_METADATA_H_

#include <optional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/hash.h"
#include "core/common/robin_map.h"
#include "core/common/robin_set.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"

namespace imp {

// Stores metadata about how the scene was authored.
//
// If present, then the node was authored from an Isf file loaded (while
// retaining metadata) or from the editor.
class SceneMetadata : public Component {
 public:
  static constexpr bool kExcludeFromEditor = true;

  struct ComponentSource {
    bool is_authored = false;
    // The last merged disabled state of all bases.
    bool disabled = false;
    std::vector<std::string> component_bytes;
  };

  void SetBaseUrl(absl::string_view base_url);

  absl::string_view GetBaseUrl() const;

  // Returns true if the node originates from a base Isf file.
  //
  // It could be that this node has a direct base (in which case GetBaseUrl will
  // not be empty) or it could be that this node is a originating within a base
  // Isf file.
  bool IsFromBase() const;

  void SetChildOfBase(bool is_child_of_base);

  // Returns true if this node originated from a child within a base Isf file.
  bool IsChildOfBase() const;

  void SetBaseDisabled(bool disabled);
  void SetBaseLocalPosition(float3 position);
  void SetBaseLocalRotation(quatf rotation);
  void SetBaseLocalScale(float3 scale);

  std::optional<bool> IsBaseDisabled();
  std::optional<float3> GetBaseLocalPosition();
  std::optional<quatf> GetBaseLocalRotation();
  std::optional<float3> GetBaseLocalScale();

  void PushBaseComponentSources(HashValue component_type, bool disabled,
                                absl::string_view component_source_bytes);

  const ComponentSource* GetBaseComponentSource(HashValue component_type);

  // Sets whether the component is considered authored, which impacts whether
  // it is serialized to the .isf file when saved in kAuthored mode and if the
  // component widget is displayed in the editor when in edit mode.
  void SetComponentAuthored(HashValue component_type, bool is_authored);

  // Returns whether the component is considered authored.
  bool IsComponentAuthored(HashValue component_type) const;

 private:
  // The base URL for this node within the Isf file.
  std::string base_url_;

  // True if this node originated from a child within a base Isf file.
  bool is_child_of_base_ = false;

  // The last state merge state of all bases when there are multiple.
  std::optional<bool> is_base_disabled_;
  std::optional<float3> base_local_position_;
  std::optional<quatf> base_local_rotation_;
  std::optional<float3> base_local_scale_;

  // The source component data from all base Isfs merged into this Isf.
  //
  // The key is the hash of the component proto type. The same key used to get
  // the handler from the SceneComponentDeserializer.
  RobinMap<HashValue, ComponentSource> base_component_sources_;

  // Component is considered authored if it was either loaded from an Isf file
  // or added directly from the editor. Otherwise, it is considered
  // dynamically generated. Dynamically generated components are not
  // serialized to the .isf file when saved in kAuthored mode.
  RobinSet<HashValue> authored_components_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_SCENE_METADATA_H_
