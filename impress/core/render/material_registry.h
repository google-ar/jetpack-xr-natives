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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_MATERIAL_REGISTRY_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_MATERIAL_REGISTRY_H_

#include <optional>
#include <string>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "core/materials/material.h"
#include "core/view/framework/render/material_definition.proto.imp.h"

namespace imp {

// Caches materials loaded from AssetManager.
class MaterialRegistry {
 public:
  MaterialRegistry() = default;

  // Registers a material with a url. Takes ownership of the material.
  // Optionally accepts a MaterialDefinition (stored only in DEV builds).
  void RegisterMaterial(
      absl::string_view url, OwnedMaterialPtr material,
      std::optional<MaterialDefinition> definition = std::nullopt);

  // Retrieves a borrowed reference to the material if it exists.
  BorrowedMaterialPtr GetMaterial(absl::string_view url) const;

  // Retrieves a pointer to the stored MaterialDefinition if it exists.
  const MaterialDefinition* GetMaterialDefinition(absl::string_view url) const;

  // Returns true if the material is already registered.
  bool HasMaterial(absl::string_view url) const;

 private:
  struct MaterialEntry {
    OwnedMaterialPtr material;
    std::optional<MaterialDefinition> definition;
  };

  absl::flat_hash_map<std::string, MaterialEntry> materials_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_MATERIAL_REGISTRY_H_
