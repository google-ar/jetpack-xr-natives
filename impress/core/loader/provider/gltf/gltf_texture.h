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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_TEXTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_TEXTURE_H_

#include <cstdint>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_attribute.h"
#include "core/loader/provider/gltf/gltf_geometry.h"
#include "core/loader/provider/gltf/gltf_helpers.h"

namespace imp::loader::details::provider_gltf {

// Loops through all the textures defined on the GltfModel, extracts the image
// pixel data / samplers, and adds them to the model builder.
absl::Status ProcessTextureInfoFromNode(
    LoadedModelBuilder& model_builder, const GltfModel& model,
    const std::vector<imp::gltf::imp_proto::Primitive>& primitives,
    const GltfPrimitiveVector<ProcessedPrimitive>& processed_primitives,
    const Gltf2AttributeMask& mask,
    LoaderOptions::TextureTranscodeCompressionType compression_type,
    bool use_lite_materials);

// Returns the lookup index for the texture in the Impress LoadedModel.
// Note: this index is not the same as the TextureId in the Impress LoadedModel.
// In order to get the TextureId, call LoadedModelBuilder::GetTexture(lookup).
absl::StatusOr<uint16_t> GetTextureLookupIndex(
    const gltf::imp_proto::Texture& texture);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_TEXTURE_H_
