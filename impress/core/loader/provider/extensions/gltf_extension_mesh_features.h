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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_MESH_FEATURES_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_MESH_FEATURES_H_

#include <vector>

#include "core/loader/provider/gltf/gltf.proto.imp.h"

// This is a partial implementation of support for the EXT_mesh_features
// extension for glTF 2.0
// (https://github.com/CesiumGS/glTF/tree/3d-tiles-next/extensions/2.0/Vendor/EXT_mesh_features).
// Currently only supports loading at most 4 texture based mesh features. Does
// not load vertex attribute features or the feature table itself.
//
// If enabled, this extension will add feature id texture TextureIds to the
// primitives/parts. The pipeline for this data looks like
// 1. During gltf_geometry::ProcessPrimitives mesh_features are found on a
//    primitive; so the mesh_features texture information is collected and
//    added to the ProcessedPrimitive.
// 2. During CreateGenericMaterialsSchema, the mesh_features textures are added
//    to the material.
//
// To access the mesh_features textures, users can access the GltfRenderer's
// ModelData and get the generic material:
//
//    const imp::model::ModelData& model_data =
//        gltf_renderer->GetGltfAsset()->GetModelData();
//    auto& generic_material =
//        model_data.Materials()[imp::model::MaterialId(material_index)];
//    auto feature_id_texture = generic_material->GetFeatureIdTexture(0);
//    if (feature_id_texture.has_value()) {
//      material->GetFilamentMaterialInstance()->setParameter(
//          "featureIdTexture0", feature_id_texture->texture,
//          feature_id_texture->sampler);
//    }
namespace imp::loader::extensions {

// Returns the feature id texture gltf indices for the given primitive. If the
// extension is not enabled, this will return an empty vector and none of the
// textures will actually be loaded.
std::vector<gltf::imp_proto::Primitive::FeatureIdTexture> ResolveMeshFeatures(
    const gltf::imp_proto::Primitive& primitive);

}  // namespace imp::loader::extensions

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_MESH_FEATURES_H_
