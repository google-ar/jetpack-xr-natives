// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_BASE_MATERIAL_HANDLE_H_
#define THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_BASE_MATERIAL_HANDLE_H_

#include <cstddef>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/hash.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/proto/proto_common.h"

namespace imp {

class BaseView;

// A base class for MaterialHandle, which is used to avoid a circular dependency
// between MaterialHandle and the framework.
class BaseMaterialHandle {
 public:
  BaseMaterialHandle() = default;
  explicit BaseMaterialHandle(absl::string_view url);
  virtual ~BaseMaterialHandle() = default;

  virtual Future<absl::Status> AsyncPreload(BaseView& view);

  Material* operator->() const;
  explicit operator bool() const;

  absl::string_view GetUrl() const;
  BorrowedMaterialPtr GetMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  void AssignMaterial(absl::string_view url, BorrowedMaterialPtr material);

  static constexpr std::size_t kFieldsCount = 1;
  static constexpr int kFieldIds[] = {1};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {"{}"};
  static constexpr absl::string_view kFieldNames[] = {"url"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0]),
  };
  static constexpr absl::string_view kFieldJsonNames[] = {"url"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0]),
  };

  template <std::size_t I>
  struct FieldType;

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, BaseMaterialHandle* other,
               Args... args) {
    return v.template Visit<proto::TYPE_STRING>(cursor, 1, &url_,
                                                other ? &other->url_ : nullptr,
                                                std::forward<Args>(args)...);
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor c,
                    BaseMaterialHandle* other, Args... args) {
    if (field_id == 1) {
      c = v.template Visit<proto::TYPE_STRING>(c, 1, &url_,
                                               other ? &other->url_ : nullptr,
                                               std::forward<Args>(args)...);
    }
    return c;
  }

 protected:
  std::string url_;
  BorrowedMaterialPtr material_;
};

template <>
struct BaseMaterialHandle::FieldType<0> {
  using Type = std::string;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_BASE_MATERIAL_HANDLE_H_
