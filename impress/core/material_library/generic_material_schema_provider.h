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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_FACTORY_H_

#include <vector>

#include "absl/status/status.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_attribute.h"
#include "core/loader/provider/gltf/gltf_geometry.h"
#include "core/loader/provider/gltf/gltf_helpers.h"

namespace imp {

absl::Status CreateGenericMaterialSchemas(
    loader::details::LoadedModelBuilder& builder,
    const loader::details::provider_gltf::GltfModel& model,
    const std::vector<imp::gltf::imp_proto::Primitive>& primitives,
    const loader::details::provider_gltf::GltfPrimitiveVector<
        loader::details::provider_gltf::ProcessedPrimitive>&
        processed_primitives,
    const loader::details::provider_gltf::Gltf2AttributeMask& mask,
    bool use_lite_materials);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_FACTORY_H_
