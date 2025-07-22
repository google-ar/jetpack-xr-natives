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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_TEXTURE_EXTERNAL_MATERIAL_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_TEXTURE_EXTERNAL_MATERIAL_H_

#include <memory>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"

// TODO : Find a better namespace for this. (e.g., imp::android)
namespace android_xr {

// Displays an external texture.
class TextureExternalMaterial : public imp::split_engine::SplitEngineMaterial {
 public:
  static imp::Future<std::unique_ptr<TextureExternalMaterial>> Create(
      imp::BaseView& view);

  ~TextureExternalMaterial() override;

  void SetTexture(imp::OwnedOrBorrowedTexturePtr texture);

 protected:
  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  TextureExternalMaterial(
      imp::BaseView& view,
      imp::split_engine::PlaceholderOrBuiltInMaterialPtr material);

  imp::OwnedOrBorrowedTexturePtr texture_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_TEXTURE_EXTERNAL_MATERIAL_H_
