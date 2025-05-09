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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_RESOURCE_BUILDERS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_RESOURCE_BUILDERS_H_

#include <cstdint>
#include <memory>
#include <optional>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/common/buffer_access.h"
#include "core/common/schemas/render_generated.h"
#include "core/image/image_contents.h"
#include "core/loader/creator/inflight_creation.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/view/base_view.h"

namespace imp::loader::details {

absl::Status FillVertexBuffer(
    BaseView& view, filament::Engine* engine, BaseVertexBufferBuilder& builder,
    const schemas::VertexBufferInfo* info, InflightCreation* inflight_creation,
    std::optional<uint8_t> vertex_access_flags = std::nullopt,
    std::optional<absl::string_view> name = std::nullopt);

absl::Status FillIndexBuffer(
    BaseView& view, filament::Engine* engine, BaseIndexBufferBuilder& builder,
    const schemas::IndexBufferInfo* info, InflightCreation* inflight_creation,
    std::optional<bool> store_index_data = std::nullopt,
    std::optional<absl::string_view> name = std::nullopt);

absl::Status FillMorphTargetBuffer(BaseView& view, filament::Engine* engine,
                                   BaseMorphTargetBufferBuilder& builder,
                                   const schemas::MorphTargetBufferInfo* info);

// Build*: allocate rendering resources
filament::Material* BuildMaterial(filament::Engine* engine,
                                  const BufferAccess& compiled_material);

filament::TextureSampler BuildTextureSampler(
    const schemas::TextureSampler& sampler);

// Textures are built and filled simultaneously because their image data must
// be encoded before the texture can be built.
filament::Texture* BuildAndFillTexture(
    BaseView& view, filament::Engine* engine, const schemas::TextureInfo* info,
    std::unique_ptr<image::ImageContents> image,
    InflightCreation* inflight_creation,
    std::optional<absl::string_view> name = std::nullopt);

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_RESOURCE_BUILDERS_H_
