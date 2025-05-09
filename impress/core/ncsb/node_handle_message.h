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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODEHANDLEMESSAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODEHANDLEMESSAGE_H_

#include <cstddef>
#include <cstdint>

#include "absl/strings/string_view.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/hash.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/proto_common.h"

// Template specializations of imp::proto::ProtoMessage provide
// serialization/deserialization for NodeHandle.
//
// imp::NodeHandleMessage is protobuf message type with no code generated in cc
// NodeHandleMessage may only be used to create/store a NodeHandle and may not
// be directly inspected or modified.
namespace imp {
template <>
struct imp::proto::ProtoMessage<imp::NodeHandle> : public imp::NodeHandle {
  static constexpr std::size_t kFieldsCount = 1;
  static constexpr int kFieldIds[] = {1};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {
      "{}",
  };
  static constexpr absl::string_view kFieldNames[] = {"entity_id"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0])};
  static constexpr absl::string_view kFieldJsonNames[] = {"entityId"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0])};
  template <std::size_t I>
  struct FieldType;

  template <>
  struct FieldType<0> {
    using Type = int32_t;
  };

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor c, imp::NodeHandle* other, Args... args) {
    int32_t entity_id = utils::Entity::smuggle(GetEntity());
    int32_t entity_id_other =
        other ? utils::Entity::smuggle(other->GetEntity()) : 0;
    c = v.template Visit<TYPE_SINT32>(c, 1, &entity_id,
                                      other ? &entity_id_other : nullptr,
                                      std::forward<Args>(args)...);
    return c;
  }

  // Deserialize Proto message to NodeHandle.
  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor c,
                    const imp::NodeHandle* other, Args... args) {
    int32_t entity_id = utils::Entity::smuggle(GetEntity());
    int32_t entity_id_other =
        other ? utils::Entity::smuggle(other->GetEntity()) : 0;
    c = v.template Visit<TYPE_SINT32>(c, 1, &entity_id,
                                      other ? &entity_id_other : nullptr,
                                      std::forward<Args>(args)...);
    *this = NodeHandle(utils::Entity::import(entity_id));
    return c;
  }

  explicit ProtoMessage(NodeHandle const& rhs) : NodeHandle(rhs) {}

  // Promote assignment operator from base class NodeHandle to enable inplace
  // deserialization.
  imp::proto::ProtoMessage<imp::NodeHandle>& operator=(NodeHandle const& rhs) {
    NodeHandle::operator=(rhs);  // invoke base class assignment operator.
    return *this;
  }
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODEHANDLEMESSAGE_H_
