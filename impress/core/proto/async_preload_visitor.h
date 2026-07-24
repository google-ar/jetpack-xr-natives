/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_ASYNC_PRELOAD_VISITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_ASYNC_PRELOAD_VISITOR_H_

#include <cstddef>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/copyable_ptr.h"
#include "core/common/one_of.h"
#include "core/common/optional_with_default.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_traits.h"

namespace imp {

class BaseView;

class AsyncPreloadVisitor {
 public:
  AsyncPreloadVisitor(BaseView& view, Future<absl::Status>& out_result)
      : view(view), out_result(out_result) {}

  template <typename T>
  static constexpr bool StateRequiresAsyncPreload() {
    return imp::proto_traits::StateRequiresAsyncPreload<T>::value;
  }

  template <int field_type, typename T>
  int Visit(int ptr, int field_id, T* field, T* other, bool optional = false) {
    if constexpr (imp::proto_traits::internal::StateHasAsyncPreloadFuncInternal<
                      T>(0)) {
      // T itself has an AsyncPreload function, so we just call it and combine
      // the result with the out_result.
      out_result = out_result.Combine(field->AsyncPreload(view));
    } else if constexpr (imp::proto::HasFields<T>::value) {
      if constexpr (StateRequiresAsyncPreload<T>()) {
        // T is a proto message and has fields that require async preload, so we
        // need to visit it.
        AsyncPreloadVisitor::Preload(*field, view, out_result);
      }
    }
    return ptr;
  }

  template <int field_type, typename T>
  int Visit(int ptr, int field_id, std::optional<T>* field,
            std::optional<T>* other, bool optional = false) {
    if (field->has_value()) {
      Visit<field_type>(ptr, field_id, &(**field), static_cast<T*>(nullptr),
                        true);
    }
    return ptr;
  }

  template <int field_type, typename T, const auto* DefaultValuePointer>
  int Visit(int ptr, int field_id,
            OptionalWithDefault<T, DefaultValuePointer>* field,
            OptionalWithDefault<T, DefaultValuePointer>* other,
            bool optional = false) {
    if (field->HasValue()) {
      Visit<field_type>(ptr, field_id, &field->MutableValue(),
                        static_cast<T*>(nullptr), true);
    }
    return ptr;
  }

  template <int field_type, typename T>
  int Visit(int ptr, int field_id, CopyablePtr<T>* field, CopyablePtr<T>* other,
            bool optional = false) {
    if (*field) {
      Visit<field_type>(ptr, field_id, field->get(), static_cast<T*>(nullptr),
                        optional);
    }
    return ptr;
  }

  template <int field_type, proto::RepeatedMergeStrategy merge_type, typename T>
  int Visit(int ptr, int field_id, std::vector<T>* field, std::vector<T>* other,
            bool optional = false) {
    for (auto& value : *field) {
      Visit<field_type>(ptr, 0, &value, static_cast<T*>(nullptr), optional);
    }
    return ptr;
  }

  template <int key_type, int value_type, typename K, typename V>
  int Visit(int ptr, int field_id, std::map<K, V>* field, std::map<K, V>* other,
            bool optional = false) {
    for (auto& [key, value] : *field) {
      Visit<value_type>(ptr, 0, &value, static_cast<V*>(nullptr), optional);
    }
    return ptr;
  }

  template <typename Proto>
  int VisitStandardProto(int ptr, int field_id, Proto* proto, Proto* other) {
    return ptr;
  }

  template <typename... Tags, typename FieldType, FieldType... field_types,
            size_t... I>
  int VisitOneOf(int ptr, imp::OneOf<Tags...>* field,
                 imp::OneOf<Tags...>* other, absl::string_view field_name,
                 std::integer_sequence<FieldType, field_types...> seq,
                 const std::vector<int>& variant_field_ids,
                 std::index_sequence<I...> seq2) {
    constexpr FieldType types_array[] = {field_types...};
    (
        [this, field, other, &ptr, &variant_field_ids]() {
          if (!field->template Holds<Tags>()) {
            return;
          }

          int variant_field_id = variant_field_ids[I];
          constexpr FieldType field_type = types_array[I];
          ptr = Visit<static_cast<int>(field_type)>(
              ptr, variant_field_id, field->template GetIf<Tags>(),
              (other && other->template Holds<Tags>())
                  ? other->template GetIf<Tags>()
                  : nullptr,
              /*optional=*/true);
        }(),
        ...);
    return ptr;
  }

  template <typename T, typename FieldType, FieldType... field_types>
  int VisitVariant(
      int ptr, T* field, T* other, absl::string_view field_name,
      std::integer_sequence<int, field_types...> variant_field_types,
      const std::vector<int>& variant_field_ids) {
    return ptr;
  }

  int Unknown(int ptr) { return ptr; }

  template <typename T>
  static Future<absl::Status> Preload(T& state, BaseView& view) {
    Future<absl::Status> result = Future<absl::Status>(absl::OkStatus());
    Preload(state, view, result);
    return result;
  }

  template <typename T>
  static void Preload(T& state, BaseView& view, Future<absl::Status>& result) {
    AsyncPreloadVisitor visitor{view, result};
    if constexpr (imp::proto::HasFields<T>::value) {
      state.Visit(visitor, 0, static_cast<T*>(nullptr));
    }
  }

 private:
  BaseView& view;
  Future<absl::Status>& out_result;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_ASYNC_PRELOAD_VISITOR_H_
