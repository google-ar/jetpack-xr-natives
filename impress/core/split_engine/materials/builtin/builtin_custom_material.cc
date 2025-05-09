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

#include "core/split_engine/materials/builtin/builtin_custom_material.h"

#include <utility>

#include "core/common/small_source_location.h"
#include "core/materials/material.h"

namespace imp::split_engine {

BuiltInCustomMaterial::BuiltInCustomMaterial(OwnedMaterialPtr material)
    : material_(std::move(material)) {}

BorrowedMaterialPtr BuiltInCustomMaterial::GetMaterialInternal(
    SmallSourceLocation loc) const {
  return material_.Borrow(loc);
}

}  // namespace imp::split_engine
