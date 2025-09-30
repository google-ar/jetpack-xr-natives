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

#include "core/ncsb/scene_metadata.h"

#include <optional>
#include <string>

#include "absl/strings/string_view.h"
#include "core/common/hash.h"
#include "core/math/quat.h"
#include "core/math/vec.h"

namespace imp {

void SceneMetadata::SetBaseUrl(absl::string_view base_url) {
  if (!base_url_.empty()) {
    return;
  }

  base_url_ = base_url;
}

absl::string_view SceneMetadata::GetBaseUrl() const { return base_url_; }

bool SceneMetadata::IsFromBase() const {
  return !base_url_.empty() || is_child_of_base_;
}

void SceneMetadata::SetBaseDisabled(bool disabled) {
  is_base_disabled_ = disabled;
}

void SceneMetadata::SetChildOfBase(bool is_child_of_base) {
  is_child_of_base_ = is_child_of_base;
}

bool SceneMetadata::IsChildOfBase() const { return is_child_of_base_; }

void SceneMetadata::SetBaseLocalPosition(float3 position) {
  base_local_position_ = position;
}
void SceneMetadata::SetBaseLocalRotation(quatf rotation) {
  base_local_rotation_ = rotation;
}
void SceneMetadata::SetBaseLocalScale(float3 scale) {
  base_local_scale_ = scale;
}

std::optional<bool> SceneMetadata::IsBaseDisabled() {
  return is_base_disabled_;
}

std::optional<float3> SceneMetadata::GetBaseLocalPosition() {
  return base_local_position_;
}
std::optional<quatf> SceneMetadata::GetBaseLocalRotation() {
  return base_local_rotation_;
}
std::optional<float3> SceneMetadata::GetBaseLocalScale() {
  return base_local_scale_;
}

void SceneMetadata::PushBaseComponentSources(
    HashValue component_type, bool disabled,
    absl::string_view component_source_bytes) {
  ComponentSource& component_source = base_component_sources_[component_type];
  component_source.disabled = disabled;
  component_source.component_bytes.push_back(
      std::string(component_source_bytes));
}

const SceneMetadata::ComponentSource* SceneMetadata::GetBaseComponentSource(
    HashValue component_type) {
  auto itr = base_component_sources_.find(component_type);
  if (itr == base_component_sources_.end()) {
    return nullptr;
  }
  return &itr->second;
}

void SceneMetadata::SetComponentAuthored(HashValue component_type,
                                         bool is_authored) {
  if (is_authored) {
    authored_components_.insert(component_type);
  } else {
    authored_components_.erase(component_type);
  }
}

bool SceneMetadata::IsComponentAuthored(HashValue component_type) const {
  return authored_components_.contains(component_type);
}

}  // namespace imp
