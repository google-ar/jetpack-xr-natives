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

#include "core/scene_handles/material_handle.h"

#include <string>
#include <utility>

#include "absl/strings/string_view.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"

namespace imp {

MaterialHandle::MaterialHandle(absl::string_view url) : url_(url) {}

Material* MaterialHandle::operator->() const { return material_.operator->(); }

MaterialHandle::operator bool() const { return material_ != nullptr; }

absl::string_view MaterialHandle::GetUrl() const { return url_; }

BorrowedMaterialPtr MaterialHandle::GetMaterial(SmallSourceLocation loc) const {
  if (!material_) {
    return BorrowedMaterialPtr();
  }
  return material_.WithNewLocation(loc);
}

void MaterialHandle::AssignMaterial(absl::string_view url,
                                    BorrowedMaterialPtr material) {
  url_ = url;
  material_ = std::move(material);
}

}  // namespace imp
