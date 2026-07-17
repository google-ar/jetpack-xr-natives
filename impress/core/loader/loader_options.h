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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_OPTIONS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_OPTIONS_H_

#include <cstdint>

namespace imp::loader {

struct LoaderOptions {
  // Accessibility to vertex attributes.
  // LINT.IfChange(VertexAccessFlags)
  enum VertexAccessFlags : uint8_t {
    kNone = 0,
    kPosition = 1 << 0,
    kTangent = 1 << 1,
    kDefault = kNone
  };
  // LINT.ThenChange(
  //     //depot/google3/third_party/impress/core/view/framework/assets/gltf_asset.h:VertexAccessFlags
  // )

  // Types of compressed texture formats which can be used by the filament
  // runtime.
  // LINT.IfChange(TranscodeCompressionType)
  // Specifies what types of compressed texture formats the loader is able to
  // decode into.
  enum class TextureTranscodeCompressionType {
    Unknown,
    AstcAndEtc,
    EtcOnly,
  };
  // LINT.ThenChange(
  //     //depot/google3/third_party/arcore/java/com/google/ar/sceneform/assets/Loader.java:TranscodeCompressionType
  // )

  TextureTranscodeCompressionType compression_type =
      TextureTranscodeCompressionType::Unknown;

  // Specifies the vertex attributes, whose data is available on CPU, so
  // that their are accessible for the users.
  uint8_t vertex_access_flags = kDefault;
  // Specifies if the glTF is being loaded using lite materials.
  bool use_lite_materials = false;
  // Specifies if the glTF being loaded should exclude excess nodes that don't
  // impact how the glTF is visually rendered.
  bool exclude_excess_nodes = false;
  // Specifies if the glTF being loaded should have its baked shadow planes
  // removed, if it has any.
  bool remove_shadow_planes = false;
  // Specifies if the texture asset API should be used.
  bool enable_use_texture_asset_api = false;
};

}  // namespace imp::loader

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_OPTIONS_H_
