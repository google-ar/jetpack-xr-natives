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

#include "core/ncsb/node.h"

#include <cassert>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/fixed_array.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/node_attachment_manager.h"
#include "core/ncsb/node_children_iterator.h"
#include "core/ncsb/node_controller.h"
#include "core/ncsb/path_manager.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"

namespace imp {
using TransformInstance = ::filament::TransformManager::Instance;

Node::Node(utils::Entity entity) : entity_(entity) {
  // If this entity is valid, then try to get the NodeController and cache it.
  if (!entity_.isNull()) {
    node_controller_ = imp_internal::NodeAttachmentManager::Get(entity_);
  }
}

Node::Node(utils::Entity entity, imp_internal::NodeController* node_controller)
    : entity_(entity), node_controller_(node_controller) {
  
}

absl::string_view Node::GetName() const { return node_controller_->GetName(); }

NodeHandle Node::FindByName(absl::string_view name) {
  return GetView().GetPathManager().FindDescendantOrSelfIf(
      NodeHandle(*this),
      [name](NodeHandle node) { return node->GetName() == name; });
}

void Node::SetName(absl::string_view name) {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetName(GetEntity(), name);
  }
  node_controller_->SetName(name);
}

void Node::SetEnabled(bool enabled) {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetEnabled(GetEntity(), enabled);
  }
  node_controller_->SetEnabled(enabled);
}

bool Node::IsEnabled() const { return node_controller_->IsEnabled(); }

bool Node::IsActive() const { return node_controller_->IsActive(); }

bool Node::IsRoot() const { return node_controller_->IsRoot(); }

#if IMP_RUNTIME(DEV)
void Node::SetAsEditorStaging(bool is_editor_staging) {
  node_controller_->SetAsEditorStaging(is_editor_staging);
}

bool Node::IsEditorStaging() const {
  return node_controller_->IsEditorStaging();
}
#endif

NodeHandle Node::CreateChildNode() const {
  NodeHandle node = GetView().CreateNode();
  NodeHandle this_node(*this);
  node->SetParentInternal(this_node, SetParentMode::kKeepLocalTransform);
  return node;
}

void Node::SetGroups(absl::Span<const absl::string_view> group_names) {
  node_controller_->SetGroups(group_names);
}

void Node::SetGroupsFromVector(const std::vector<std::string>& group_names) {
  SetGroups(absl::FixedArray<absl::string_view>(group_names.begin(),
                                                group_names.end()));
}

void Node::AddToGroup(absl::string_view group_name) {
  node_controller_->AddToGroup(group_name);
}

void Node::RemoveFromGroup(absl::string_view group_name) {
  node_controller_->RemoveFromGroup(group_name);
}

void Node::ClearGroups() { node_controller_->SetGroups(std::nullopt); }

std::vector<std::string> Node::GetGroups() const {
  return node_controller_->GetGroups();
}

bool Node::IsInGroup(absl::string_view group_name) const {
  return node_controller_->IsInGroup(group_name);
}

void Node::SetParent(NodeHandle parent) {
  SetParentInternal(parent, SetParentMode::kKeepLocalTransform);
}

void Node::SetParentKeepWorldTransform(NodeHandle parent) {
  SetParentInternal(parent, SetParentMode::kKeepWorldTransform);
}

void Node::SetParentInternal(NodeHandle parent, SetParentMode mode) {
  auto& tm = GetTransformManager();
  TransformInstance instance = tm.getInstance(GetEntity());

  // Return early if the parent isn't changing.
  if (parent.GetEntity() == tm.getParent(instance)) {
    return;
  }

  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetParent(
        GetEntity(),
        parent ? parent->GetEntity()
               : utils::Entity::import(split_engine::kInvalidEntityId));
  }

  if (parent) {
    if (&GetView() != &(parent->GetView())) {
      IMP_LOG(imp::FATAL) << "Cannot set the parent to a node attached to a different "
                    "imp::View.";
    }

    TransformInstance parentInstance = tm.getInstance(parent->GetEntity());

    // ImpView owns this enum, so use an exhaustive switch.
    // Falls through to the same behavior as kKeepLocalTransform if the enum
    // value doesn't match either enumeration.
    // (broken link)
    switch (mode) {
      case SetParentMode::kKeepLocalTransform:
        // Do nothing.
        break;
      case SetParentMode::kKeepWorldTransform:
        // Find what the local transform should be relative to the new parent
        // to maintain the same world transform after the parent changes and
        // then assign the new transform.
        if (!GetView().IsPreciseTranslationEnabled()) {
          mat4f inverse_parent_world_transform =
              inverse(tm.getWorldTransform(parentInstance));
          mat4f world_transform = tm.getWorldTransform(instance);
          mat4f local_transform =
              inverse_parent_world_transform * world_transform;
          SetLocalTrs(local_transform);
        } else {
          mat4 inverse_parent_world_transform =
              inverse(tm.getWorldTransformAccurate(parentInstance));
          mat4 world_transform = tm.getWorldTransformAccurate(instance);
          mat4 local_transform =
              inverse_parent_world_transform * world_transform;
          SetLocalTrsPrecise(local_transform);
        }

        break;
    }

    tm.setParent(instance, parentInstance);
  } else {
    // ImpView owns this enum, so use an exhaustive switch.
    // Falls through to the same behavior as kKeepLocalTransform if the enum
    // value doesn't match either enumeration.
    // (broken link)
    switch (mode) {
      case SetParentMode::kKeepLocalTransform:
        // Do nothing.
        break;
      case SetParentMode::kKeepWorldTransform:
        // Since the node no longer has a parent, what was the world transform
        // just becomes the local transform.
        mat4f world_transform = tm.getWorldTransform(instance);
        SetLocalTrs(world_transform);
        break;
    }

    tm.setParent(instance, TransformInstance());
  }

  node_controller_->OnParentChanged();
}

NodeHandle Node::GetParent() const {
  auto& tm = GetTransformManager();
  TransformInstance instance = tm.getInstance(GetEntity());

  // If there is no parent, this will be a null entity.
  utils::Entity parent_entity = tm.getParent(instance);

  // Make sure the entity is attached to the view.
  // Return a NodeHandle to the entity, may be invalid if there is no parent.
  return GetView().AttachEntityToView(parent_entity);
}

std::vector<NodeHandle> Node::GetChildren() const {
  NodeChildrenRange range = GetChildrenRange();
  std::vector<NodeHandle> result;
  result.reserve(range.GetCount());
  result.assign(range.begin(), range.end());
  return result;
}

NodeChildrenRange Node::GetChildrenRange() const {
  auto& tm = GetTransformManager();
  TransformInstance instance = tm.getInstance(GetEntity());
  return NodeChildrenRange(tm, instance);
}

void Node::SetLocalPosition(const float3& position) {
  auto& tm = GetTransformManager();
  TransformInstance instance = tm.getInstance(GetEntity());

  mat4f transform = tm.getTransform(instance);
  transform[3][0] = position.x;
  transform[3][1] = position.y;
  transform[3][2] = position.z;

  // filament does clear the high precision data when setting with low precision
  // data so we don't need to branch here.
  SetFilamentTransformInternal(instance, transform);
}

void Node::SetLocalPositionPrecise(const double3& position) {
  auto& tm = GetTransformManager();
  TransformInstance instance = tm.getInstance(GetEntity());

  // This might return an invalid transform if accurate translation mode is not
  // enabled before this call. However, this should be fine since we are going
  // to overwrite the translation.
  mat4 transform = tm.getTransformAccurate(instance);
  transform[3][0] = position.x;
  transform[3][1] = position.y;
  transform[3][2] = position.z;

  bool was_precise_translation_enabled =
      GetView().IsPreciseTranslationEnabled();
  if (!was_precise_translation_enabled) {
    // Open a local transform transaction to avoid recomputing world transform
    // when enabling precise translation mode.
    tm.openLocalTransformTransaction();
    GetView().SetPreciseTranslationEnabled(true);
  }

  SetFilamentTransformInternal(instance, transform);

  if (!was_precise_translation_enabled) {
    tm.commitLocalTransformTransaction();
  }
}

float3 Node::GetLocalPosition() const { return GetLocalTrs()[3].xyz; }

double3 Node::GetLocalPositionPrecise() const {
  return GetLocalTrsPrecise()[3].xyz;
}

void Node::SetLocalScale(const float3& scale) {
  node_controller_->SetLocalScale(scale);

  // We need to branch here since if high precision translation mode is enabled,
  // getting the low precision translation data and setting it back might result
  // in loss of precision.
  if (!GetView().IsPreciseTranslationEnabled()) {
    Transform<float> transform = GetLocalTransform();
    transform.scale = scale;
    SetFilamentTransformInternal(transform);
  } else {
    Transform<double> transform = GetLocalTransformPrecise();
    transform.scale = scale;
    SetFilamentTransformInternal(transform);
  }
}

float3 Node::GetLocalScale() const { return node_controller_->GetLocalScale(); }

void Node::SetLocalRotation(const quatf& rotation) {
  node_controller_->SetLocalRotation(rotation);

  if (!GetView().IsPreciseTranslationEnabled()) {
    Transform<float> transform = GetLocalTransform();
    transform.rotation = rotation;

    SetFilamentTransformInternal(transform);
  } else {
    Transform<double> transform = GetLocalTransformPrecise();
    transform.rotation = quat(rotation);

    SetFilamentTransformInternal(transform);
  }
}

quatf Node::GetLocalRotation() const {
  return node_controller_->GetLocalRotation();
}

float3 Node::GetLocalForward() const { return GetLocalRotation() * kForward; }

void Node::SetLocalForward(const float3& look_direction) {
  SetLocalForward(look_direction, kUp);
}

void Node::SetLocalForward(const float3& look_direction,
                           const float3& up_direction) {
  auto transform = Transform<float>(
      mat4f::lookAt(float3(0, 0, 0), look_direction, up_direction));
  SetLocalRotation(transform.rotation);
}

void Node::SetWorldPosition(const float3& position) {
  if (NodeHandle parent = GetParent()) {
    float3 new_position = parent->LocalFromWorldPoint(position);
    SetLocalPosition(new_position);
  } else {
    SetLocalPosition(position);
  }
}

void Node::SetWorldPositionPrecise(const double3& position) {
  if (NodeHandle parent = GetParent()) {
    double3 new_position = parent->LocalFromWorldPointPrecise(position);
    SetLocalPositionPrecise(new_position);
  } else {
    SetLocalPositionPrecise(position);
  }
}

float3 Node::GetWorldPosition() const {
  return Transform<float>(GetWorldTrs()).translation;
}

double3 Node::GetWorldPositionPrecise() const {
  return Transform<double>(GetWorldTrsPrecise()).translation;
}

void Node::SetWorldRotation(const quatf& rotation) {
  if (NodeHandle parent = GetParent()) {
    quatf parent_rotation = parent->GetWorldRotation();
    quatf parent_rotation_inverted = {-parent_rotation.xyz, parent_rotation.w};
    SetLocalRotation(parent_rotation_inverted * rotation);
  } else {
    SetLocalRotation(rotation);
  }
}

quatf Node::GetWorldRotation() const {
  auto transform = Transform<float>(GetWorldTrs());
  return transform.rotation;
}

void Node::SetWorldScale(const float3& scale) {
  if (NodeHandle parent = GetParent()) {
    auto transform = GetLocalTransform();
    transform.scale = {1.0f};
    mat4f trs = transform.AsMat4();

    trs = parent->GetWorldTrs() * trs;
    mat4f inverse_trs = inverse(trs);
    trs = inverse_trs * mat4f::scaling(scale);

    transform = Transform<float>(trs);

    SetLocalScale(transform.scale);
  } else {
    SetLocalScale(scale);
  }
}

float3 Node::GetWorldScale() const {
  auto transform = Transform<float>(GetWorldTrs());
  return transform.scale;
}

float3 Node::GetWorldForward() const { return GetWorldRotation() * kForward; }

void Node::SetWorldForward(const float3& look_direction) {
  SetWorldForward(look_direction, kUp);
}

void Node::SetWorldForward(const float3& look_direction,
                           const float3& up_direction) {
  auto transform = Transform<float>(
      mat4f::lookAt(float3(0, 0, 0), look_direction, up_direction));
  SetWorldRotation(transform.rotation);
}

float3 Node::WorldFromLocalPoint(const float3& position) const {
  const mat4f& world_trs = GetWorldTrs();
  return (world_trs * position).xyz;
}

double3 Node::WorldFromLocalPointPrecise(const double3& position) const {
  const mat4& world_trs = GetWorldTrsPrecise();
  return (world_trs * position).xyz;
}

float3 Node::WorldFromLocalVector(const float3& vector) const {
  const mat4f& world_trs = GetWorldTrs();
  return (world_trs * float4{vector, 0.0f}).xyz;
}

double3 Node::WorldFromLocalVectorPrecise(const double3& vector) const {
  const mat4& world_trs = GetWorldTrsPrecise();
  return (world_trs * double4{vector, 0.0}).xyz;
}

float3 Node::LocalFromWorldPoint(const float3& position) const {
  const mat4f& world_trs = GetWorldTrs();
  mat4f world_trs_inverse = inverse(world_trs);
  return (world_trs_inverse * position).xyz;
}

double3 Node::LocalFromWorldPointPrecise(const double3& position) const {
  const mat4& world_trs = GetWorldTrsPrecise();
  mat4 world_trs_inverse = inverse(world_trs);
  return (world_trs_inverse * position).xyz;
}

float3 Node::LocalFromWorldVector(const float3& vector) const {
  const mat4f& world_trs = GetWorldTrs();
  mat4f world_trs_inverse = inverse(world_trs);
  return (world_trs_inverse * float4{vector, 0.0f}).xyz;
}

double3 Node::LocalFromWorldVectorPrecise(const double3& vector) const {
  const mat4& world_trs = GetWorldTrsPrecise();
  mat4 world_trs_inverse = inverse(world_trs);
  return (world_trs_inverse * double4{vector, 0.0}).xyz;
}

const mat4f& Node::GetLocalTrs() const {
  auto& tm = GetTransformManager();
  // Returns an invalid instance if the entity has no transform.
  TransformInstance instance = tm.getInstance(GetEntity());

  // Returns an identity matrix if the instance is invalid.
  return tm.getTransform(instance);
}

Transform<float> Node::GetLocalTransform() const {
  return Transform<float>(GetLocalTrs()[3].xyz,
                          node_controller_->GetLocalRotation(),
                          node_controller_->GetLocalScale());
}

void Node::SetLocalTransform(const Transform<float>& transform) {
  node_controller_->SetLocalRotation(transform.rotation);
  node_controller_->SetLocalScale(transform.scale);
  SetFilamentTransformInternal(transform);
}

Transform<double> Node::GetLocalTransformPrecise() const {
  return Transform<double>(GetLocalTrsPrecise()[3].xyz,
                           node_controller_->GetLocalRotation(),
                           node_controller_->GetLocalScale());
}

void Node::SetLocalTransformPrecise(const Transform<double>& transform) {
  node_controller_->SetLocalRotation(static_cast<quatf>(transform.rotation));
  node_controller_->SetLocalScale(transform.scale);

  auto& tm = GetTransformManager();
  bool was_precise_translation_enabled =
      GetView().IsPreciseTranslationEnabled();
  if (!was_precise_translation_enabled) {
    // Open a local transform transaction to avoid recomputing world transform
    // when enabling precise translation mode.
    tm.openLocalTransformTransaction();
    GetView().SetPreciseTranslationEnabled(true);
  }

  SetFilamentTransformInternal(transform);

  if (!was_precise_translation_enabled) {
    tm.commitLocalTransformTransaction();
  }
}

mat4 Node::GetLocalTrsPrecise() const {
  auto& tm = GetTransformManager();
  // Returns an invalid instance if the entity has no transform.
  TransformInstance instance = tm.getInstance(GetEntity());

  // Returns an identity matrix if the instance is invalid.
  return tm.getTransformAccurate(instance);
}

void Node::SetLocalTrs(const mat4f& trs) {
  Transform<float> transform(trs);

  node_controller_->SetLocalRotation(transform.rotation);
  node_controller_->SetLocalScale(transform.scale);
  TransformInstance instance = GetTransformManager().getInstance(GetEntity());
  SetFilamentTransformInternal(instance, trs);
}

void Node::SetLocalTrsPrecise(const mat4& trs) {
  Transform<double> transform(trs);

  node_controller_->SetLocalRotation(static_cast<quatf>(transform.rotation));
  node_controller_->SetLocalScale(transform.scale);

  auto& tm = GetTransformManager();
  TransformInstance instance = tm.getInstance(GetEntity());
  bool was_precise_translation_enabled =
      GetView().IsPreciseTranslationEnabled();
  if (!was_precise_translation_enabled) {
    // Open a local transform transaction to avoid recomputing world transform
    // when enabling precise translation mode.
    tm.openLocalTransformTransaction();
    GetView().SetPreciseTranslationEnabled(true);
  }

  SetFilamentTransformInternal(instance, trs);

  if (!was_precise_translation_enabled) {
    tm.commitLocalTransformTransaction();
  }
}

const mat4f& Node::GetWorldTrs() const {
  auto& tm = GetTransformManager();
  // Returns an invalid instance if the entity has no transform.
  TransformInstance instance = tm.getInstance(GetEntity());

  // Returns an identity matrix if the instance is invalid.
  return tm.getWorldTransform(instance);
}

mat4 Node::GetWorldTrsPrecise() const {
  auto& tm = GetTransformManager();
  TransformInstance instance = tm.getInstance(GetEntity());

  return tm.getWorldTransformAccurate(instance);
}

void Node::SetWorldTrs(const mat4f& trs) {
  const auto& world_from_self = trs;
  if (NodeHandle parent = GetParent()) {
    const auto world_from_parent = parent->GetWorldTrs();
    const auto self_from_parent = inverse(world_from_self) * world_from_parent;
    SetLocalTrs(inverse(self_from_parent));
  } else {
    // Without a parent, parent_from_self == world_from_self
    SetLocalTrs(world_from_self);
  }
}

void Node::SetWorldTrsPrecise(const mat4& trs) {
  const auto& world_from_self = trs;
  if (NodeHandle parent = GetParent()) {
    const auto world_from_parent = parent->GetWorldTrsPrecise();
    const auto self_from_parent = inverse(world_from_self) * world_from_parent;
    SetLocalTrsPrecise(inverse(self_from_parent));
  } else {
    // Without a parent, parent_from_self == world_from_self
    SetLocalTrsPrecise(world_from_self);
  }
}

Invocable<void()> Node::Remember(Holdable holdable) {
  return node_controller_->Remember(std::move(holdable));
}

void Node::SetFilamentTransformInternal(const Transform<float>& transform) {
  TransformInstance instance = GetTransformManager().getInstance(GetEntity());
  SetFilamentTransformInternal(instance, transform.AsMat4());
}

void Node::SetFilamentTransformInternal(const Transform<double>& transform) {
  filament::TransformManager& tm = GetTransformManager();
  TransformInstance instance = tm.getInstance(GetEntity());
  bool was_precise_translation_enabled =
      GetView().IsPreciseTranslationEnabled();
  if (!was_precise_translation_enabled) {
    // Open a local transform transaction to avoid recomputing world transform
    // when enabling precise translation mode.
    tm.openLocalTransformTransaction();
    GetView().SetPreciseTranslationEnabled(true);
  }

  SetFilamentTransformInternal(instance, transform.AsMat4());

  if (!was_precise_translation_enabled) {
    tm.commitLocalTransformTransaction();
  }
}

void Node::SetFilamentTransformInternal(
    ::filament::TransformManager::Instance instance, const mat4f& transform) {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetLocalTransform(GetEntity(), transform);
  }
  GetTransformManager().setTransform(instance, transform);
}

void Node::SetFilamentTransformInternal(
    ::filament::TransformManager::Instance instance, const mat4& transform) {
  if (split_engine::SplitEngineSerializer* serializer =
          GetView().GetSplitEngineSerializer()) {
    serializer->SetLocalTransform(GetEntity(), transform);
  }
  GetTransformManager().setTransform(instance, transform);
}

}  // namespace imp
