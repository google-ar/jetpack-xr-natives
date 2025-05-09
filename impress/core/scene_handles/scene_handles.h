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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLES_H_
#define THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLES_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/common/hash.h"
#include "core/common/type_traits.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/proto_common.h"
#include "core/scene_handles/scene_handle_helper.h"
#include "core/scene_handles/scene_handle_interface.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

// Represents a node that can be deserialized from the NodeSceneHandle proto.
//
// Used when loading a .isf file to assign references between nodes within the
// file.
//
// See the NodeSceneHandle proto:
//   depot/google3/third_party/impress/core/ncsb/node_scene_handle.proto
//
// See an example usage:
//   depot/google3/third_party/impress/samples/isf
//
// TODO: Add error handling when saving handles in a scene that
// are invalid because they aren't part of the scene or for some other reason
// like no unique_id or name.
//
// TODO: Add support for saving scene handles using automatically
// generated unique_ids when there are duplicate names within the scene.
class NodeSceneHandle : public NodeHandle, public SceneHandleInterface {
 public:
  using SceneHandleInterface::kTypeUrlHash;

  NodeSceneHandle();
  explicit NodeSceneHandle(SceneHandleInterface::Identifier identifier);
  NodeSceneHandle(SceneHandleInterface::Identifier identifier, NodeHandle node);

  absl::Status AssignSceneHandleForIdentifier(
      NodeHandle identified_node, NodeHandle attached_node) override;
  const SceneHandleInterface::Identifier& GetIdentifier() override;
  std::string ToSceneHandleString() const override;

  virtual std::string GetTypeName() const override;
  NodeHandle GetSceneNode() const override;
  void AssignSceneHandleForNode(NodeHandle node) override;
  bool CanAssignSceneHandleForNode(NodeHandle node) const override;

  static constexpr std::size_t kFieldsCount = 2;
  static constexpr int kFieldIds[] = {1, 2};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {"{}", "{}"};
  static constexpr absl::string_view kFieldNames[] = {"name", "unique_id"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0]),
      imp::Hash(kFieldNames[1]),
  };
  static constexpr absl::string_view kFieldJsonNames[] = {"name", "uniqueId"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0]),
      imp::Hash(kFieldJsonNames[1]),
  };
  template <std::size_t I>
  struct FieldType;

  template <>
  struct FieldType<0> {
    using Type = std::string;
  };
  template <>
  struct FieldType<1> {
    using Type = int32_t;
  };

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, NodeSceneHandle* other,
               Args... args) {
    // Calling ResolveIdentifier here is important because it ensures that
    // the identifier is updated if necessary prior to visiting the fields.
    SceneHandleInterface::Identifier* val = &ResolveIdentifier();
    SceneHandleInterface::Identifier* other_identifier =
        other ? &other->ResolveIdentifier() : nullptr;

    return scene_handle_helper_.VisitIdentifier(
        v, cursor, val, other_identifier, std::forward<Args>(args)...);
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor c, NodeSceneHandle* other,
                    Args... args) {
    return scene_handle_helper_.VisitIdentifierField(
        field_id, v, c,
        other ? &other->scene_handle_helper_.GetIdentifier() : nullptr,
        std::forward<Args>(args)...);
  }

 private:
  SceneHandleInterface::Identifier& ResolveIdentifier();

  SceneHandleHelper scene_handle_helper_;
};

// Represents a component that can be deserialized from the ComponentSceneHandle
// proto.
//
// Used when loading a .isf file to assign references to a component on a node
// within the file.
//
// Example Proto:
//
// message FooState {
//  option (imp.native_include) =
//      "third_party/path/to/bar.h";
//
//  ComponentSceneHandle bar_comp = 1
//      [(imp.template_type) = "::imp::Bar"];
// }
//
template <typename T>
class ComponentSceneHandle : public ComponentHandle<T>,
                             public SceneHandleInterface {
 public:
  using SceneHandleInterface::kTypeUrlHash;

  // Declare Component type T as a dependency so that when a
  // ComponentSceneHandle is included in a component's State proto the
  // ComponentSceneHandle's type is automatically used as a dependency. This
  // way, the component doesn't need to explicit list T as a dependency to
  // guarantee that the ComponentSceneHandle is setup first.
  using IsfDependencies = IsfDependencies<T>;

  ComponentSceneHandle() {}
  explicit ComponentSceneHandle(SceneHandleInterface::Identifier identifier)
      : scene_handle_helper_(identifier) {}
  ComponentSceneHandle(SceneHandleInterface::Identifier identifier,
                       ComponentHandle<T> comp)
      : ComponentHandle<T>(comp), scene_handle_helper_(identifier) {}

  // Constructs ComponentSceneHandle with just the component.
  // Warning: if multiple nodes share the same name as the component's node,
  // then it is unspecified which node will be referenced.
  ComponentSceneHandle(ComponentHandle<T> comp)
      : ComponentHandle<T>(comp),
        scene_handle_helper_(SceneHandleInterface::Identifier(
            std::string(comp.Get()->GetNode()->GetName()))) {}

  absl::Status AssignSceneHandleForIdentifier(
      NodeHandle identified_node, NodeHandle attached_node) override;
  const SceneHandleInterface::Identifier& GetIdentifier() override;
  std::string ToSceneHandleString() const override;

  virtual std::string GetTypeName() const override;
  NodeHandle GetSceneNode() const override;
  void AssignSceneHandleForNode(NodeHandle node) override;
  bool CanAssignSceneHandleForNode(NodeHandle node) const override;
  static constexpr std::size_t kFieldsCount = 2;
  static constexpr int kFieldIds[] = {1, 2};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {"{}", "{}"};
  static constexpr absl::string_view kFieldNames[] = {"name", "unique_id"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0]),
      imp::Hash(kFieldNames[1]),
  };
  static constexpr absl::string_view kFieldJsonNames[] = {"name", "uniqueId"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0]),
      imp::Hash(kFieldJsonNames[1]),
  };
  template <std::size_t I>
  struct FieldType;

  template <>
  struct FieldType<0> {
    using Type = std::string;
  };
  template <>
  struct FieldType<1> {
    using Type = int32_t;
  };

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, ComponentSceneHandle* other,
               Args... args) {
    // Calling ResolveIdentifier here is important because it ensures that
    // the identifier is updated if necessary prior to visiting the fields.
    SceneHandleInterface::Identifier* val = &ResolveIdentifier();
    SceneHandleInterface::Identifier* other_identifier =
        other ? &other->ResolveIdentifier() : nullptr;

    return scene_handle_helper_.VisitIdentifier(
        v, cursor, val, other_identifier, std::forward<Args>(args)...);
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor c,
                    ComponentSceneHandle* other, Args... args) {
    return scene_handle_helper_.VisitIdentifierField(
        field_id, v, c,
        other ? &other->scene_handle_helper_.GetIdentifier() : nullptr,
        std::forward<Args>(args)...);
  }

 private:
  SceneHandleInterface::Identifier& ResolveIdentifier();

  SceneHandleHelper scene_handle_helper_;
};

// Represents a node that can be deserialized from the GltfNodeSceneHandle
// proto.
//
// Used when loading a .isf file to assign references to a node within a glTF
// filed loaded by a .isf file.
//
// See the NodeSceneHandle proto:
//   depot/google3/third_party/impress/core/ncsb/node_scene_handle.proto
//
// See an example usage:
//   depot/google3/third_party/impress/samples/isf
class GltfNodeSceneHandle : public NodeHandle, public SceneHandleInterface {
 public:
  using SceneHandleInterface::kTypeUrlHash;

  // Declare GltfRenderer as a dependency to guarantee that glTFs are loaded
  // prior to the GltfNodeSceneHandle being assigned.
  using IsfDependencies = IsfDependencies<GltfRenderer>;

  GltfNodeSceneHandle();

  explicit GltfNodeSceneHandle(absl::string_view gltf_node_name);

  // The identifier represents which node within a .isf file has the glTF
  // containing the glTF node we're looking for. If unset, then we look for the
  // glTF node on the node containing the GltfNodeSceneHandle.
  GltfNodeSceneHandle(SceneHandleInterface::Identifier identifier,
                      absl::string_view gltf_node_name);

  // The identifier represents which node within a .isf file has the glTF
  // containing the glTF node we're looking for.
  // If unset, then we look for the glTF node on the node containing the
  // GltfNodeSceneHandle.
  GltfNodeSceneHandle(SceneHandleInterface::Identifier identifier,
                      absl::string_view gltf_node_name, NodeHandle node);

  std::string GetGltfNodeName() const;

  absl::Status AssignSceneHandleForIdentifier(
      NodeHandle identified_node, NodeHandle attached_node) override;
  const SceneHandleInterface::Identifier& GetIdentifier() override;
  std::string ToSceneHandleString() const override;

  virtual std::string GetTypeName() const override;
  NodeHandle GetSceneNode() const override;
  void AssignSceneHandleForNode(NodeHandle node) override;
  bool CanAssignSceneHandleForNode(NodeHandle node) const override;

  static constexpr std::size_t kFieldsCount = 3;
  static constexpr int kFieldIds[] = {1, 2, 3};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {"{}", "{}",
                                                                   "{}"};
  static constexpr absl::string_view kFieldNames[] = {"name", "unique_id",
                                                      "gltf_node_name"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0]),
      imp::Hash(kFieldNames[1]),
      imp::Hash(kFieldNames[2]),
  };
  static constexpr absl::string_view kFieldJsonNames[] = {"name", "uniqueId",
                                                          "gltfNodeName"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0]),
      imp::Hash(kFieldJsonNames[1]),
      imp::Hash(kFieldJsonNames[2]),
  };
  template <std::size_t I>
  struct FieldType;
  template <>
  struct FieldType<0> {
    using Type = std::string;
  };
  template <>
  struct FieldType<1> {
    using Type = int32_t;
  };
  template <>
  struct FieldType<2> {
    using Type = std::string;
  };

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, GltfNodeSceneHandle* other,
               Args... args) {
    cursor = scene_handle_helper_.VisitIdentifier(
        v, cursor, &scene_handle_helper_.GetIdentifier(),
        other ? &other->scene_handle_helper_.GetIdentifier() : nullptr,
        std::forward<Args>(args)...);
    return v.template Visit<proto::TYPE_STRING>(
        cursor, 3, &gltf_node_name_, other ? &other->gltf_node_name_ : nullptr,
        std::forward<Args>(args)...);
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor c,
                    GltfNodeSceneHandle* other, Args... args) {
    c = scene_handle_helper_.VisitIdentifierField(
        field_id, v, c,
        other ? &other->scene_handle_helper_.GetIdentifier() : nullptr,
        std::forward<Args>(args)...);
    if (field_id == 3) {
      c = v.template Visit<proto::TYPE_STRING>(
          c, 3, &gltf_node_name_, other ? &other->gltf_node_name_ : nullptr,
          std::forward<Args>(args)...);
    }
    return c;
  }

 private:
  SceneHandleHelper scene_handle_helper_;

  // The name of the node within a glTF file that this handle should be assigned
  // to.
  std::string gltf_node_name_;
};

template <typename T>
absl::Status ComponentSceneHandle<T>::AssignSceneHandleForIdentifier(
    NodeHandle identified_node, NodeHandle attached_node) {
  if (absl::holds_alternative<absl::monostate>(
          scene_handle_helper_.GetIdentifier())) {
    // No identifier assigned.
    return absl::OkStatus();
  }

  MP_RETURN_IF_ERROR(scene_handle_helper_.RequireIdentifiedNode(identified_node));

  ComponentHandle<T> comp = identified_node->GetComponent<T>();
  if (!comp) {
    return absl::UnavailableError(absl::StrFormat(
        "ComponentSceneHandle cannot find component of type %s "
        "on node with identifier %s",
        type_traits::kTypeName<T>, scene_handle_helper_.GetIdentifierString()));
  }

  *this = ComponentSceneHandle<T>(scene_handle_helper_.GetIdentifier(), comp);
  return absl::OkStatus();
}

template <typename T>
const SceneHandleInterface::Identifier&
ComponentSceneHandle<T>::GetIdentifier() {
  return ResolveIdentifier();
}

template <typename T>
SceneHandleInterface::Identifier& ComponentSceneHandle<T>::ResolveIdentifier() {
  scene_handle_helper_.UpdateIdentifier(GetSceneNode());
  return scene_handle_helper_.GetIdentifier();
}

template <typename T>
std::string ComponentSceneHandle<T>::ToSceneHandleString() const {
  return scene_handle_helper_.GetIdentifierString();
}

template <typename T>
std::string ComponentSceneHandle<T>::GetTypeName() const {
  return std::string(type_traits::kTypeName<T>);
}

template <typename T>
NodeHandle ComponentSceneHandle<T>::GetSceneNode() const {
  if (ComponentHandle<T>::IsValid()) {
    return ComponentHandle<T>::Get()->GetNode();
  }
  return {};
}

template <typename T>
void ComponentSceneHandle<T>::AssignSceneHandleForNode(NodeHandle node) {
  if (!node) {
    *this = ComponentSceneHandle<T>();
    return;
  }

  *this = ComponentSceneHandle<T>(node->GetComponent<T>());
}

template <typename T>
bool ComponentSceneHandle<T>::CanAssignSceneHandleForNode(
    NodeHandle node) const {
  if (!node) {
    return false;
  }

  if (node->GetName().empty()) {
    return false;
  }

  auto comp = node->GetComponent<T>();
  if (!comp) {
    return false;
  }

  return true;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLES_H_
