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

#include "core/loader/provider/extensions/gltf_extension_mesh_features.h"

#include <vector>

#include "core/loader/provider/gltf/gltf.proto.imp.h"

namespace imp::loader::extensions {

std::vector<gltf::imp_proto::Primitive::FeatureIdTexture> ResolveMeshFeatures(
    const imp::gltf::imp_proto::Primitive& primitive) {
  std::vector<gltf::imp_proto::Primitive::FeatureIdTexture> feature_id_textures;
  if (primitive.extensions.mesh_features.has_value()) {
    for (const gltf::imp_proto::Primitive::FeatureId& feature_id :
         primitive.extensions.mesh_features->feature_ids) {
      if (feature_id.texture.has_value()) {
        feature_id_textures.push_back(*feature_id.texture);
      }
    }
  }
  return feature_id_textures;
}

}  // namespace imp::loader::extensions
