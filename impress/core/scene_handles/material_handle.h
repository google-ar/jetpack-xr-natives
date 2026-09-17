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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_MATERIAL_HANDLE_H_
#define THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_MATERIAL_HANDLE_H_

#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/scene_handles/base_material_handle.h"

namespace imp {

class BaseView;

// Represents a material that can be deserialized from the MaterialHandle
// proto.
//
// Used when loading a .isf file to assign references to materials in the
// AssetManager.
//
// Note: this is an embedded proto, it should be used as a ISF field and should
// not be instantiated directly.
// WARNING: This is work in progress and not ready to be used yet.
class MaterialHandle : public BaseMaterialHandle {
 public:
  using BaseMaterialHandle::BaseMaterialHandle;

  MaterialHandle() = default;
  explicit MaterialHandle(absl::string_view url) : BaseMaterialHandle(url) {}

  Future<absl::Status> AsyncPreload(BaseView& view) override;

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, MaterialHandle* other, Args... args) {
    return BaseMaterialHandle::Visit(v, cursor, other,
                                     std::forward<Args>(args)...);
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor c, MaterialHandle* other,
                    Args... args) {
    return BaseMaterialHandle::VisitField(field_id, v, c, other,
                                          std::forward<Args>(args)...);
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SCENE_HANDLES_MATERIAL_HANDLE_H_
