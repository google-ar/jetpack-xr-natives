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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_VERIFICATION_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_VERIFICATION_H_

#include "absl/status/status.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/vector.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/typed_set_vector.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"

namespace imp {
namespace loader {
namespace optional_features {

absl::Status VerifyAndGetModel(
    BufferAccess&& storage, FlatBufferAccess<schemas::LoadedModel>* out_access);

absl::Status VerifyModelNestedData(const schemas::LoadedModel* model);

absl::Status VerifyEntities(const schemas::LoadedModel* loaded_model);

absl::Status VerifyTexture(const schemas::TextureInfo* texture_info);

absl::Status VerifySamplers(
    const flatbuffers::Vector<flatbuffers::Offset<schemas::TextureSampler>>*
        samplers);

absl::Status VerifyParts(
    const schemas::LoadedModel* loaded_model,
    const flatbuffers::Vector<flatbuffers::Offset<schemas::PartInfo>>* parts);

absl::Status VerifyBoneData(TypedSetVector<model::BoneData>& bones);

absl::Status VerifyEntityData(
    TypedSetVector<model::ModelData::EntityData>& entities);

absl::Status VerifyJointData(
    TypedSetVector<model::ModelData::JointData>& joints);

}  // namespace optional_features
}  // namespace loader
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_VERIFICATION_H_
