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

#include "core/render/material_registry.h"

#include <optional>
#include <string>
#include <utility>

#include "absl/strings/string_view.h"
#include "core/materials/material.h"
#include "core/view/framework/render/material_definition.proto.imp.h"

namespace imp {

void MaterialRegistry::RegisterMaterial(
    absl::string_view url, OwnedMaterialPtr material,
    std::optional<MaterialDefinition> definition) {
  materials_[std::string(url)] = {std::move(material), std::move(definition)};
}

BorrowedMaterialPtr MaterialRegistry::GetMaterial(absl::string_view url) const {
  auto it = materials_.find(url);
  if (it != materials_.end()) {
    return it->second.material.Borrow();
  }
  return nullptr;
}

const MaterialDefinition* MaterialRegistry::GetMaterialDefinition(
    absl::string_view url) const {
  auto it = materials_.find(url);
  if (it != materials_.end()) {
    if (it->second.definition.has_value()) {
      return &it->second.definition.value();
    }
  }
  return nullptr;
}

bool MaterialRegistry::HasMaterial(absl::string_view url) const {
  return materials_.contains(url);
}

}  // namespace imp
