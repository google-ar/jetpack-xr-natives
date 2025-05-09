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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLE_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLE_HELPER_H_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/types/variant.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/proto_common.h"
#include "core/scene_handles/scene_handle_interface.h"

namespace imp {

// Helper class used by implementations of SceneHandleInterface for implementing
// functionality common to all scene handles.
struct SceneHandleHelper {
  using Identifier = SceneHandleInterface::Identifier;

  SceneHandleHelper();
  explicit SceneHandleHelper(Identifier identifier);

  // Returns the scene handle's identifier.
  Identifier& GetIdentifier();

  // Returns a string representing the identifier.
  std::string GetIdentifierString() const;

  // Helper functioning for returning an error if there is no identified node.
  absl::Status RequireIdentifiedNode(NodeHandle identified_node) const;

  void UpdateIdentifier(NodeHandle scene_node);

  // Serializes proto message representing a scene handle's identifier.
  //
  // See
  //   third_party/impress/core/scene_handles/scene_handles.proto
  template <typename Visitor, typename Cursor, typename... Args>
  static Cursor VisitIdentifier(Visitor& v, Cursor cursor, Identifier* val,
                                Identifier* other, Args... args) {
    constexpr bool kHasVisitVariantIdentifierFn =
        ::imp::proto_traits::kHasVisitVariantFunction<
            Visitor, Cursor, Identifier, std::integer_sequence<int, 0>>;
    cursor = !kHasVisitVariantIdentifierFn && val->index() == 1
                 ? v.template Visit<proto::TYPE_STRING>(
                       cursor, 1, absl::get_if<1>(val), absl::get_if<1>(other),
                       std::forward<Args>(args)...)
                 : cursor;
    cursor = !kHasVisitVariantIdentifierFn && val->index() == 2
                 ? v.template Visit<proto::TYPE_SINT32>(
                       cursor, 2, absl::get_if<2>(val), absl::get_if<2>(other),
                       std::forward<Args>(args)...)
                 : cursor;
    if constexpr (kHasVisitVariantIdentifierFn) {
      cursor = v.VisitVariant(cursor, val, other, "identifier",
                              std::integer_sequence<signed, proto::TYPE_STRING,
                                                    proto::TYPE_SINT32>{},
                              {1, 2}, std::forward<Args>(args)...);
    }
    return cursor;
  }

  // Deerializes proto message representing a scene handle's identifier.
  //
  // See
  //   third_party/impress/core/scene_handles/scene_handles.proto
  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitIdentifierField(int field_id, Visitor& v, Cursor c,
                              Identifier* other, Args... args) {
    switch (field_id) {
      case 1:
        if (identifier_.index() != 1) {
          identifier_.template emplace<1>();
        }
        return v.template Visit<proto::TYPE_STRING>(
            c, 1, absl::get_if<1>(&identifier_), absl::get_if<1>(other),
            std::forward<Args>(args)...);
      case 2:
        if (identifier_.index() != 2) {
          identifier_.template emplace<2>();
        }
        return v.template Visit<proto::TYPE_SINT32>(
            c, 2, absl::get_if<2>(&identifier_), absl::get_if<2>(other),
            std::forward<Args>(args)...);
      default:
        return v.Unknown(c, field_id, std::forward<Args>(args)...);
    }
  }

 private:
  Identifier identifier_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_SCENE_HANDLE_HELPER_H_
