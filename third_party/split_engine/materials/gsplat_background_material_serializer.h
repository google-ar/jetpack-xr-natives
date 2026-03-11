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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_BACKGROUND_MATERIAL_SERIALIZER_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_BACKGROUND_MATERIAL_SERIALIZER_H_

#include <sys/types.h>

#include <memory>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"

namespace android_xr {

// A material wrapper for the built-in GsplatBackgroundMaterial to support Split
// Engine.
//
// TODO: (broken link) - BuiltInMaterialGsplatBackground is a placeholder. Once we
// have custom materials that should replace this material. Alternatively if a
// stencil buffer is used this will not be needed.
//
// Note: This is the split-engine app side of BuiltInGsplatMaterial.

class GsplatBackgroundMaterialSerializer
    : public imp::split_engine::SplitEngineMaterial {
 public:
  static imp::Future<std::unique_ptr<GsplatBackgroundMaterialSerializer>>
  Create(imp::BaseView& view);

  ~GsplatBackgroundMaterialSerializer() override;

  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  GsplatBackgroundMaterialSerializer(
      imp::BaseView& view,
      imp::split_engine::PlaceholderOrBuiltInMaterialPtr material);
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_GSPLAT_BACKGROUND_MATERIAL_SERIALIZER_H_
