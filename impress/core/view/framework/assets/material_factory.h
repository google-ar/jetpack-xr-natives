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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_MATERIAL_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_MATERIAL_FACTORY_H_

#include <vector>

#include "absl/base/attributes.h"
#include "absl/strings/string_view.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_registry.h"
#include "core/view/base_view.h"
#include "core/view/framework/render/material.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/utils/asset.h"

namespace imp {

// Creates Materials asynchronously.
class MaterialFactory {
 public:
  explicit MaterialFactory(BaseView* view) : view_(view) {}

  // Loads the Material asynchronously from string.
  Future<MaterialPtr> LoadMaterial(absl::string_view asset_url);

  // Loads the Material asynchronously from AssetDefinition.
  Future<MaterialPtr> LoadMaterial(const AssetDefinition& asset_definition);

  Future<MaterialPtr> LoadMaterial(
      const MaterialDefinition& material_definition);

  // Creates a Material from a previously loaded AssetPtr.
  MaterialPtr CreateMaterial(const AssetPtr<MaterialAsset>& material_asset);

  // Creates a Material from a filament material. The method calls to create a
  // filament material instance from the given material. The caller should make
  // sure that the original material will still be kept and the created instance
  // does not outlive it. This method might be helpful mainly when creating
  // imp materials from filament directly and not from imp::MaterialAssets.
  ABSL_DEPRECATED("Use the const reference version instead.")
  MaterialPtr CreateMaterial(const filament::Material* material);
  MaterialPtr CreateMaterial(const filament::Material& material);

  // Wraps a Filament MaterialInstance with an imp Material, thus giving
  // it an official owner.
  MaterialPtr WrapMaterial(filament::MaterialInstance* material);

  // Sets the given material parameters on the given material.
  static Future<absl::Status> SetMaterialParameters(
      BaseView& view, Material* material,
      const std::vector<MaterialDefinition::Parameter>& parameters);

 private:
  BaseView* view_;
};

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_MATERIAL_FACTORY_H_
