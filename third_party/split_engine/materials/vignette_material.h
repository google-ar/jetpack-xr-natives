/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_VIGNETTE_MATERIAL_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_VIGNETTE_MATERIAL_H_

#include <memory>
#include <optional>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "imp.h"

namespace android_xr {

// Displays a screen-space vignette effect to the user's view.
// This uses a Split Engine Built-in material.
class VignetteMaterial : public imp::split_engine::SplitEngineMaterial {
 public:
  static imp::Future<std::unique_ptr<VignetteMaterial>> Create(
      imp::BaseView& view);

  void SetColor(const imp::float4& color);
  void SetCoverage(float coverage);
  void SetFeather(float feather);

 protected:
  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  VignetteMaterial(imp::BaseView& view,
                   imp::split_engine::PlaceholderOrBuiltInMaterialPtr material);

  std::optional<imp::float4> color_;
  std::optional<float> coverage_;
  std::optional<float> feather_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_VIGNETTE_MATERIAL_H_
