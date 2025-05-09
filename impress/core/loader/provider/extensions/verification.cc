// Copyright 2024 Google LLC
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

#include "core/loader/provider/extensions/verification.h"

#include <cstddef>
#include <cstdint>
#include <utility>

#include "absl/status/status.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/vector.h"
#include "core/common/buffer_access.h"
#include "core/common/enum_flags.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_tree.h"
#include "core/loader/details/bundle_resource_helpers.h"
#include "core/loader/details/loaded_model_fb.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace loader {
namespace optional_features {

absl::Status VerifyAndGetModel(
    BufferAccess&& storage,
    FlatBufferAccess<schemas::LoadedModel>* out_access) {
  return details::VerifyAndGetModel(std::move(storage),
                                    details::VerifyOptions::All, out_access);
}

absl::Status VerifyModelNestedData(const schemas::LoadedModel* model) {
  return details::VerifyModelNestedData(model);
}

absl::Status VerifyPartInfo(const schemas::LoadedModel* model,
                            const schemas::PartInfo* part) {
  if (part->vertex_buffer() >= model->vertex_buffers()->size()) {
    return Error("Invalid vertex buffer");
  }

  const uint16_t index_buffer = part->index_buffer();
  if (index_buffer >= model->index_buffers()->size()) {
    return Error("Invalid index buffer");
  }

  const schemas::IndexBufferInfo* index_info =
      model->index_buffers()->Get(index_buffer);
  const size_t index_count = details::GetIndexCount(index_info);
  if (part->index_offset() + part->index_count() > index_count) {
    return Error("Invalid index range");
  }

  if (part->material() > model->materials()->size()) {
    return Error(
        "Invalid material with %u part material index and %u total materials",
        part->material(), model->materials()->size());
  }

  return NoError();
}

absl::Status VerifyEntityInfo(const schemas::LoadedModel* model,
                              const schemas::EntityInfo* entity) {
  if (entity->runtime() && !details::VerifyFlags(entity->runtime()->flags())) {
    return Error("Invalid flags");
  }

  if (entity->skin() < -1 ||
      (entity->skin() >= 0 &&
       entity->skin() >= flatbuffers::VectorLength(model->skins()))) {
    return Error("Invalid skin");
  }

  if (entity->bone() >=
      flatbuffers::VectorLength(model->skeleton()->child_counts())) {
    return Error("Invalid bone");
  }

  for (const schemas::PartInfo* part : *entity->parts()) {
    MP_RETURN_IF_ERROR(VerifyPartInfo(model, part));
  }

  return NoError();
}

absl::Status VerifyEntities(const schemas::LoadedModel* loaded_model) {
  const auto* entities = loaded_model->entity_graph()->entities();
  const size_t entities_count = entities->size();
  if (!entities_count) return NoError();
  size_t found_children = 0;
  for (size_t i = 0; i < entities_count; ++i) {
    const schemas::EntityInfo* entity = entities->Get(i);
    const uint16_t num_children = entity->num_children();

    found_children += num_children;

    if (i + num_children > entities_count) {
      return Error("Invalid children");
    }

    MP_RETURN_IF_ERROR(VerifyEntityInfo(loaded_model, entity));
  }

  if (found_children >= entities_count) {
    return Error("Invalid entity tree (found %llu children, total count %llu)",
                 found_children, entities_count);
  }

  return NoError();
}

absl::Status VerifyTexture(const schemas::TextureInfo* texture_info) {
  if (!details::VerifyFlags(texture_info->flags())) {
    return Error("Invalid flags");
  }

  const Flags<schemas::TextureInfoFlags> flags(texture_info->flags());

  if (flags.Test(schemas::TextureInfoFlags::IsSrgb) &&
      flags.Test(schemas::TextureInfoFlags::IsR11G11B10)) {
    return Error("Invalid format");
  }

  return NoError();
}

absl::Status VerifySamplers(
    const flatbuffers::Vector<flatbuffers::Offset<schemas::TextureSampler>>*
        samplers) {
  for (const auto* sampler : *samplers) {
    if (!details::VerifyEnum(sampler->min_filter()) ||
        !details::VerifyEnum(sampler->mag_filter()) ||
        !details::VerifyEnum(sampler->wrap_mode_s()) ||
        !details::VerifyEnum(sampler->wrap_mode_t()) ||
        !details::VerifyEnum(sampler->wrap_mode_r()) ||
        !details::VerifyEnum(sampler->compare_mode()) ||
        !details::VerifyEnum(sampler->compare_func())) {
      return Error("Invalid enum");
    }
  }

  return NoError();
}

absl::Status VerifyParts(
    const schemas::LoadedModel* loaded_model,
    const flatbuffers::Vector<flatbuffers::Offset<schemas::PartInfo>>* parts) {
  for (const schemas::PartInfo* part : *parts) {
    MP_RETURN_IF_ERROR(VerifyPartInfo(loaded_model, part));
  }

  return NoError();
}

absl::Status VerifyBoneData(TypedSetVector<model::BoneData>& bones) {
  return TypedDagTools<model::BoneId>::VerifyGraph(
      bones.Span<model::BoneData::kNumChildren>(),
      bones.Span<model::BoneData::kParent>(),
      bones.Span<model::BoneData::kFirstChild>(),
      bones.Span<model::BoneData::kNextSibling>());
}

absl::Status VerifyEntityData(
    TypedSetVector<model::ModelData::EntityData>& entities) {
  if (entities.empty()) {
    return absl::OkStatus();
  }

  return TypedDagTools<model::ModelData::EntityId>::VerifyGraph(
      entities.Span<model::ModelData::EntityData::kNumChildren>(),
      entities.Span<model::ModelData::EntityData::kParent>(),
      entities.Span<model::ModelData::EntityData::kFirstChild>(),
      entities.Span<model::ModelData::EntityData::kNextSibling>());
}

absl::Status VerifyJointData(
    TypedSetVector<model::ModelData::JointData>& joints) {
  return TypedDagTools<model::ModelData::JointId>::VerifyGraph(
      joints.Span<model::ModelData::JointData::kNumChildren>(),
      joints.Span<model::ModelData::JointData::kParent>(),
      joints.Span<model::ModelData::JointData::kFirstChild>(),
      joints.Span<model::ModelData::JointData::kNextSibling>());
}

}  // namespace optional_features
}  // namespace loader
}  // namespace imp
