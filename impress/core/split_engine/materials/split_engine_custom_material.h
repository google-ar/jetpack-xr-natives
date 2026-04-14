// Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Material.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

// A SplitEngineCustomMaterial is used when an Impress application defines a
// material that is used through Split Engine. The material is sent in
// source form to the Split Engine renderer side. This class provides access to
// the material for the app and is responsible for serializing material changes
// to the Split Engine host.
class SplitEngineCustomMaterial : public SplitEngineMaterial {
 public:
  // Request a custom filament material from the backend.
  static Future<absl::Status> RequestCustomFilamentMaterial(
      BaseView& view, absl::string_view material_source,
      filament::Material* filament_material,
      const MaterialPreCompileOptions& precompile_options =
          MaterialAsset::kDefaultPreCompileOptions);

  explicit SplitEngineCustomMaterial(BaseView& view, OwnedMaterialPtr material);

  ~SplitEngineCustomMaterial() override;

  bool HasParameter(absl::string_view name) override;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_H_
