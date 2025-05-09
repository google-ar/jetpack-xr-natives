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
#include <string>
#include <utility>
#include <variant>

#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "imp.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

// Displays a 3D texture with support for various formats and parameters.
class PhotosTexture3DMaterial : public imp::split_engine::SplitEngineMaterial {
 public:
  static imp::Future<std::unique_ptr<PhotosTexture3DMaterial>> Create(
      imp::BaseView& view);

  // Sets a parameter value.  T must be one of the supported FlatBuffer types.
  // Caller is responsible for ensuring that the type matches the parameter name
  // as defined in //third_party/split_engine/schemas/split_engine_material.fbs
  template <typename Param>
  void SetParameter(const typename Param::ValueType& value) {
    parameters_[Param::kName] = imp::split_engine::Pack(value);
    MarkParametersDirty();
  }

  // Specialization for textures, taking an OwnedOrBorrowedTexturePtr.
  void SetTexture(const std::string& name,
                  imp::OwnedOrBorrowedTexturePtr texture) {
    parameters_[name] = std::move(texture);
    MarkParametersDirty();
  }

 protected:
  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  PhotosTexture3DMaterial(
      imp::BaseView& view,
      imp::split_engine::PlaceholderOrBuiltInMaterialPtr material);

  // Variant storing all supported parameter types for this material.
  using ParameterValue =
      std::variant<std::monostate, android_xr::schemas::Bool,
                   android_xr::schemas::Float, android_xr::schemas::Float2,
                   android_xr::schemas::Float3, android_xr::schemas::Float4,
                   android_xr::schemas::Mat3f, imp::OwnedOrBorrowedTexturePtr>;

  absl::flat_hash_map<std::string, ParameterValue> parameters_;

  template <typename T>
  const T* ParamOrNull(const std::string& name) const {
    auto itr = parameters_.find(name);
    if (itr != parameters_.end()) {
      if (std::holds_alternative<T>(itr->second)) {
        return std::get_if<T>(&itr->second);
      } else {
        LOG(FATAL) << "[photosxr] Set value of parameter " << name
                   << " is not of type " << typeid(T).name();
      }
    }
    return nullptr;
  }

  void WriteTexture(
      const std::string& name,
      flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>& offset,
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_PHOTOS_TEXTURE_3D_MATERIAL_H_
