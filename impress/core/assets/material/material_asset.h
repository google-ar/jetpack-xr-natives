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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_MATERIAL_MATERIAL_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_MATERIAL_MATERIAL_ASSET_H_

#include <memory>
#include <utility>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Material.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {

// MaterialAsset is thin wrapper around filament::Material.
class MaterialAsset {
 public:
  static const MaterialPreCompileOptions& kDefaultPreCompileOptions;

  static filament::Material* BuildMaterial(
      filament::Engine& engine, const uint8_t* data, size_t size,
      const MaterialPreCompileOptions& material_pre_compile_options =
          kDefaultPreCompileOptions);

  static Future<std::unique_ptr<MaterialAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future,
      MaterialPreCompileOptions material_pre_compile_options =
          kDefaultPreCompileOptions);

  MaterialAsset(BaseView* view, filament::Material* material);
  ~MaterialAsset();

  MaterialAsset(const MaterialAsset&) = delete;
  MaterialAsset& operator=(const MaterialAsset& rhs) = delete;

  MaterialAsset(MaterialAsset&& rhs)
      : view_(rhs.view_), material_(rhs.material_) {
    rhs.material_ = nullptr;
  }
  MaterialAsset& operator=(MaterialAsset&& rhs) {
    view_ = rhs.view_;
    std::swap(material_, rhs.material_);
    return *this;
  }

  // Returns the underlying filament::Material*.
  filament::Material* GetFilamentMaterial() const { return material_; }

 private:
  BaseView* view_;
  filament::Material* material_ = nullptr;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_MATERIAL_MATERIAL_ASSET_H_
