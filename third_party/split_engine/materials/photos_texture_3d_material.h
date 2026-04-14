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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_PHOTOS_TEXTURE_3D_MATERIAL_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_PHOTOS_TEXTURE_3D_MATERIAL_H_

#include <memory>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "imp.h"
#include "split_engine/materials/photos_texture_3d_material_params.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace android_xr {

// Displays a 3D texture with support for various formats and parameters.
class PhotosTexture3DMaterial
    : public imp::split_engine::SplitEngineBuiltinMaterial {
 public:
  static imp::Future<std::unique_ptr<PhotosTexture3DMaterial>> Create(
      imp::BaseView& view, const PhotosTexture3DMaterialParams& params);

  ~PhotosTexture3DMaterial() override;

  void MarkParametersDirty() {
    SplitEngineBuiltinMaterial::MarkParametersDirty();
  }

 protected:
  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  PhotosTexture3DMaterial(imp::BaseView& view,
                          const PhotosTexture3DMaterialParams& params,
                          imp::OwnedMaterialPtr material);

  template <typename Param>
  void WriteTexture(
      flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>& offset,
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const;

  const PhotosTexture3DMaterialParams& params_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_PHOTOS_TEXTURE_3D_MATERIAL_H_
