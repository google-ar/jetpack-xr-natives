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

#include "core/loader/creator/model_creator.h"

#include <sys/types.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/vector.h"
#include "core/async/future.h"
#include "core/common/bit_vector.h"
#include "core/common/data_helpers.h"
#include "core/common/enum_flags.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/hash.h"
#include "core/common/optional_error.h"
#include "core/common/paired_vector.h"
#include "core/common/robin_map.h"
#include "core/common/schemas/math_generated.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/small_source_location.h"
#include "core/common/trace.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_vector.h"
#include "core/image/image_contents.h"
#include "core/loader/creator/create_mesh_data.h"
#include "core/loader/creator/inflight_creation.h"
#include "core/loader/creator/resource_builders.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/extensions/gltf_extension_interactivity.h"
#include "core/loader/provider/extensions/interactivity/model_creator_extension.h"
#include "core/loader/provider/extensions/verification.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/generic_material_impl.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_package.h"
#include "core/material_library/material_param_value.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/math.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/model/mesh/mesh_builder.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {

// (broken link) start
using ::filament::LinearColorA;
using ::filament::Texture;
using ::imp::model::BoneChildId;
using ::imp::model::BoneData;
using ::imp::model::BoneId;
using ::imp::model::BoneLookup;
using ::imp::model::BoneParentId;
using ::imp::model::ModelData;
using ::imp::model::SkeletonData;
using AudioEmitterId = ModelData::AudioEmitterId;
using EntityData = ModelData::EntityData;
using EntityId = ModelData::EntityId;
using IndexBufferId = ModelData::IndexBufferId;
using JointChildId = ModelData::JointChildId;
using JointData = ModelData::JointData;
using JointId = ModelData::JointId;
using JointParentId = ModelData::JointParentId;
using LightPunctualData = ModelData::LightPunctualData;
using LightPunctualId = ModelData::LightPunctualId;
using MaterialConfig = ModelData::MaterialConfig;
using MaterialId = ModelData::MaterialId;
using MeshIndexDataLookup = ModelData::MeshIndexDataLookup;
using MeshVertexDataLookup = ModelData::MeshVertexDataLookup;
using MorphTargetBufferId = ModelData::MorphTargetBufferId;
using NodeHoverability = ModelData::NodeHoverability;
using NodeSelectability = ModelData::NodeSelectability;
using NodeVisibility = ModelData::NodeVisibility;
using PartData = ModelData::PartData;
using SampledJointData = ModelData::SampledJointData;
using SkinData = ModelData::SkinData;
using SkinId = ModelData::SkinId;
using SkinnedEntityData = ModelData::SkinnedEntityData;
using SkinningBufferId = ModelData::SkinningBufferId;
using VertexBufferId = ModelData::VertexBufferId;
using WeakEntityId = ModelData::WeakEntityId;
using WeakSampledJointId = ModelData::WeakSampledJointId;
using schemas::SkeletonInfo;
// (broken link) end

template <typename T>
using JointLookup = ModelData::JointLookup<T>;
template <typename T>
using MaterialLookup = ModelData::MaterialLookup<T>;
using MaterialConfig = ModelData::MaterialConfig;
using WeakSampledJointId = ModelData::WeakSampledJointId;
template <typename T>
using SampledJointLookup = ModelData::SampledJointLookup<T>;
template <typename T>
using Vector = ::flatbuffers::Vector<T>;
template <typename T>
using Offset = ::flatbuffers::Offset<T>;

namespace {

// Maximum number of vertices, global to the model.
static constexpr size_t kMaxVertexCount = 10000000;             // 10 million.
static constexpr size_t kMaxMorphTargetTextureWidth = 2048;
static constexpr size_t kMaxMorphTargetTextureSize =
    kMaxMorphTargetTextureWidth * kMaxMorphTargetTextureWidth;
static constexpr float3 kMinimumHalfExtents = float3(1.0e-3f);  // 1 mm.

ModelData::RuntimeData GetDefaultRuntimeData() {
  // All flags off; render priority of 4.
  return ModelData::RuntimeData{{}, 4};
}

ModelData::RuntimeData RuntimeDataFromInfo(const schemas::RuntimeInfo* info) {
  using schemas::RenderInfoFlags;
  Flags<RenderInfoFlags> info_flags(info->flags());

  Flags<ModelData::RenderFlags> flags =
      ToFlags(info_flags & RenderInfoFlags::DisableFrustumCulling
                  ? ModelData::RenderFlags::DisableFrustumCulling
                  : ModelData::RenderFlags::Empty) |
      ToFlags(info_flags & RenderInfoFlags::DoNotCastShadows
                  ? ModelData::RenderFlags::DoNotCastShadows
                  : ModelData::RenderFlags::Empty) |
      ToFlags(info_flags & RenderInfoFlags::DoNotReceiveShadows
                  ? ModelData::RenderFlags::DoNotReceiveShadows
                  : ModelData::RenderFlags::Empty);
  return ModelData::RuntimeData{flags, info->priority()};
}

OptionalError CreateModelEntityGraph(
    filament::Engine* engine, const schemas::LoadedModel* model,
    const schemas::EntityGraphInfo* entity_graph,
    TypedSetVector<EntityData>* out_entities) {
  const Vector<Offset<schemas::EntityInfo>>& entities =
      *entity_graph->entities();
  const Vector<uint16_t>& parents = *entity_graph->parents();
  const Vector<uint16_t>& first_children = *entity_graph->first_children();
  const Vector<uint16_t>& next_siblings = *entity_graph->next_siblings();
  size_t entity_count = entities.size();
  MP_RETURN_IF_ERROR(optional_features::VerifyEntities(model));

  for (size_t entity_index = 0; entity_index < entity_count; ++entity_index) {
    const schemas::EntityInfo* entity = entities.Get(entity_index);

    std::vector<PartData> parts;
    parts.reserve(entity->parts()->size());
    // Build parts.
    absl::c_transform(
        *entity->parts(), std::back_inserter(parts),
        [](const schemas::PartInfo* part) -> PartData {
          ModelData::MaterialsVariantsMappingLookup materials_variants_mappings;
          if (part->materials_variants_mappings()) {
            for (uint16_t material_id : *part->materials_variants_mappings()) {
              materials_variants_mappings.emplace_back(material_id);
            }
          }

          return PartData{part->name()->str(),
                          part->index_offset(),
                          part->index_count(),
                          VertexBufferId(part->vertex_buffer()),
                          IndexBufferId(part->index_buffer()),
                          MaterialId(part->material()),
                          part->original_material_index(),
                          ModelData::PrimitiveType(part->primitive_type()),
                          std::move(materials_variants_mappings),
                          SkinningBufferId(part->skinning_buffer()),
                          part->morph_target_buffer_offset(),
                          part->morph_target_buffer_count()};
        });

    std::optional<filament::Box> bounds;
    if (entity->bounds()) {
      filament::Box box;
      box.center = FromFlatbuffer(entity->bounds()->bounds()->center());
      box.halfExtent =
          max(FromFlatbuffer(entity->bounds()->bounds()->half_extent()),
              kMinimumHalfExtents);
      bounds.emplace(box);
    }

    ModelData::RuntimeData runtime_data =
        entity->runtime() ? RuntimeDataFromInfo(entity->runtime())
                          : GetDefaultRuntimeData();

    std::optional<NodeVisibility> node_visibility;
    if (entity->node_visibility()) {
      node_visibility = NodeVisibility{
          .visible =
              entity->node_visibility()->visible()
                  ? std::optional(entity->node_visibility()->visible()->value())
                  : std::nullopt};
    }

    std::optional<NodeSelectability> node_selectability;
    if (entity->node_selectability()) {
      node_selectability = NodeSelectability{
          .selectable =
              entity->node_selectability()->selectable()
                  ? std::optional(
                        entity->node_selectability()->selectable()->value())
                  : std::nullopt};
    }

    std::optional<NodeHoverability> node_hoverability;
    if (entity->node_hoverability()) {
      node_hoverability = NodeHoverability{
          .hoverable =
              entity->node_hoverability()->hoverable()
                  ? std::optional(
                        entity->node_hoverability()->hoverable()->value())
                  : std::nullopt};
    }

    std::vector<float> node_morph_target_weights;
    if (entity->node_morph_target_weights()) {
      // Optionally populate dest_weights if there is any morph target
      // data.
      node_morph_target_weights.resize(
          entity->node_morph_target_weights()->size());
      std::copy(entity->node_morph_target_weights()->begin(),
                entity->node_morph_target_weights()->end(),
                node_morph_target_weights.begin());
    }

    std::vector<float> mesh_morph_target_weights;
    if (entity->mesh_morph_target_weights()) {
      // Optionally populate dest_weights if there is any per mesh morph target
      // data.
      mesh_morph_target_weights.resize(
          entity->mesh_morph_target_weights()->size());
      std::copy(entity->mesh_morph_target_weights()->begin(),
                entity->mesh_morph_target_weights()->end(),
                mesh_morph_target_weights.begin());
    }

    out_entities->push_back(
        entity->num_children(),
        ModelData::EntityParentId(parents.Get(entity_index)),
        ModelData::EntityChildId(first_children.Get(entity_index)),
        ModelData::EntityChildId(next_siblings.Get(entity_index)),
        std::move(parts), model::BoneId(entity->bone()), SkinId(entity->skin()),
        MorphTargetBufferId(entity->morph_target_buffer()),
        std::move(node_morph_target_weights),
        std::move(mesh_morph_target_weights),
        LightPunctualId(entity->light_punctual()),
        AudioEmitterId(entity->audio_emitter()), bounds, runtime_data,
        (entity->name() ? entity->name()->str() : ""), entity->original_index(),
        entity->original_mesh_index(), entity->original_skin_index(),
        node_visibility, node_selectability, node_hoverability);
  }
  return NoError();
}

// static
Future<absl::Status> CreateModelResources(
    BaseView& view, filament::Engine* engine, const LoaderOptions options,
    const schemas::LoadedModel* model,
    std::vector<std::unique_ptr<image::ImageContents>> images,
    TypedVector<filament::VertexBuffer*>* out_vertex_buffers,
    TypedVector<filament::IndexBuffer*>* out_index_buffers,
    TypedVector<filament::MorphTargetBuffer*>* out_morph_target_buffers,
    PairedVector<OwnedTexturePtr, filament::MorphTargetBuffer*>*
        out_morph_target_uv0_textures,
    TypedVector<OwnedTexturePtr>* out_textures,
    TypedVector<GenericMaterialPtr>* out_materials,
    MeshVertexDataLookup* stored_vertex_data,
    MeshIndexDataLookup* stored_index_data,
    MaterialLookup<MaterialConfig>* out_material_configs,
    InflightCreation* out_inflight_creation,
    const MaterialPackage::MaterialCache& materials_by_params,
    absl::flat_hash_map<uint16_t, MaterialId>& material_id_lookup,
    std::optional<absl::string_view> name) {
  IMP_TRACE();
  // Validate that the total vertex count is reasonable, since it can
  // result in large memory allocations.
  size_t total_count = 0;
  for (const schemas::VertexBufferInfo* vertex_buffer_info :
       *model->vertex_buffers()) {
    const size_t count = vertex_buffer_info->vertex_count();
    if (count > kMaxVertexCount || (total_count += count) > kMaxVertexCount) {
      return Future<absl::Status>(
          absl::InvalidArgumentError("Vertex limit exceeded"));
    }
  }

  MeshBuilder mesh_builder(view);

  bool store_mesh_data =
      options.vertex_access_flags != LoaderOptions::VertexAccessFlags::kNone;

  // Create vertex buffers.
  for (const schemas::VertexBufferInfo* vertex_buffer_info :
       *model->vertex_buffers()) {
    BaseVertexBufferBuilder& vertex_buffer_builder =
        mesh_builder.CreateVertexBufferBuilder();
    absl::Status status = details::FillVertexBuffer(
        view, engine, vertex_buffer_builder, vertex_buffer_info,
        out_inflight_creation, options.vertex_access_flags, name);
    if (!status.ok()) {
      return Future<absl::Status>(status);
    }
    // Store index data if needed.
    if (store_mesh_data) {
      stored_vertex_data->emplace_back(CreateMeshVertexData(
          *vertex_buffer_info, options.vertex_access_flags));
    }
  }

  for (const schemas::MorphTargetBufferInfo* morph_target_buffer_info :
       *model->morph_target_buffers()) {
    BaseMorphTargetBufferBuilder& morph_target_buffer_builder =
        mesh_builder.CreateMorphTargetBufferBuilder();
    absl::Status status = details::FillMorphTargetBuffer(
        view, engine, morph_target_buffer_builder, morph_target_buffer_info);
    if (!status.ok()) {
      return Future<absl::Status>(status);
    }
  }

  // Create index buffers.
  for (const schemas::IndexBufferInfo* index_buffer_info :
       *model->index_buffers()) {
    BaseIndexBufferBuilder& index_buffer_builder =
        mesh_builder.CreateIndexBufferBuilder();
    bool store_index_data =
        options.vertex_access_flags != LoaderOptions::VertexAccessFlags::kNone;
    absl::Status status = details::FillIndexBuffer(
        view, engine, index_buffer_builder, index_buffer_info,
        out_inflight_creation,
        options.vertex_access_flags != LoaderOptions::VertexAccessFlags::kNone,
        name);
    if (!status.ok()) {
      return Future<absl::Status>(status);
    }
    // Store index data if needed.
    if (store_index_data) {
      stored_index_data->emplace_back(CreateMeshIndexData(*index_buffer_info));
    }
  }

  mesh_builder.Build(out_vertex_buffers, out_index_buffers,
                     out_morph_target_buffers);

  // Create optional UV morph textures to correspond with each morph target
  // buffer. Insert a nullptr for morph target buffers that do not have
  // texcoords0 data to indicate the target buffer has no UV morphs.
  for (const schemas::MorphTargetBufferInfo* morph_target_buffer_info :
       *model->morph_target_buffers()) {
    const size_t vertex_count = morph_target_buffer_info->vertex_count();
    if (vertex_count > kMaxMorphTargetTextureSize) {
      return Future<absl::Status>(absl::InvalidArgumentError(
          "Vertex limit exceeded for UV morph target texture."));
    }

    // Determine if there is any UV morph data for this target buffer and
    // validate the vertex count matches when there is.
    bool has_uv_morphs = false;
    for (const schemas::MorphTargetAttributeInfo* target :
         *morph_target_buffer_info->targets()) {
      if (target->texcoords0() != nullptr) {
        has_uv_morphs = true;
        if (target->texcoords0()->size() != vertex_count * sizeof(float2)) {
          return Future<absl::Status>(absl::InvalidArgumentError(
              "UV morph target texture data size mismatch."));
        }
      }
    }

    if (!has_uv_morphs) {
      out_morph_target_uv0_textures->emplace_back(nullptr);
      continue;
    }

    const size_t width = std::min(vertex_count, kMaxMorphTargetTextureWidth);
    const size_t height = (vertex_count + kMaxMorphTargetTextureWidth - 1) /
                          kMaxMorphTargetTextureWidth;
    const size_t num_targets = morph_target_buffer_info->targets()->size();

    // sampler2dArray: width = 2048 (max), height = ceil(vertex_count / 2048),
    // depth = num_targets. Format: RG32F (float2). Matches Filament's built-in
    // morphTarget wrapping logic.
    filament::Texture* uv_morph_tex =
        filament::Texture::Builder()
            .width(width)
            .height(height)
            .depth(num_targets)
            .levels(1)
            .sampler(filament::Texture::Sampler::SAMPLER_2D_ARRAY)
            .format(filament::Texture::InternalFormat::RG32F)
            .build(*engine);

    const size_t pixels_per_target = width * height;
    float2* uv_data = new float2[pixels_per_target * num_targets];
    std::memset(uv_data, 0, pixels_per_target * num_targets * sizeof(float2));
    for (size_t t = 0; t < num_targets; ++t) {
      const schemas::MorphTargetAttributeInfo* target =
          morph_target_buffer_info->targets()->Get(t);
      if (target->texcoords0() != nullptr) {
        std::memcpy(&uv_data[t * pixels_per_target],
                    target->texcoords0()->Data(),
                    vertex_count * sizeof(float2));
      }
    }

    filament::Texture::PixelBufferDescriptor desc(
        uv_data, pixels_per_target * num_targets * sizeof(float2),
        filament::Texture::Format::RG, filament::Texture::Type::FLOAT,
        [](void* buffer, size_t size, void* user) {
          delete[] static_cast<float2*>(buffer);
        });

    uv_morph_tex->setImage(*engine, 0, 0, 0, 0, width, height, num_targets,
                           std::move(desc));

    // TODO: (broken link) - Support SAMPLER_2D_ARRAY in Split Engine. Once
    // supported, use view.GetTextureFactory().CreateTexture() to enable
    // serialization of this UV morph texture.
    out_morph_target_uv0_textures->emplace_back(
        view.GetTextureFactory().WrapTexture(uv_morph_tex));
  }

  // Create textures.
  if (model->textures()->size() != model->images()->size() ||
      model->images()->size() != images.size()) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        "texture infos and texture contents did not match"));
  }
  for (size_t i = 0; i < model->textures()->size(); ++i) {
    const schemas::TextureInfo* texture_info = model->textures()->Get(i);
    if (absl::Status status = optional_features::VerifyTexture(texture_info);
        !status.ok()) {
      return Future<absl::Status>(status);
    }

    Texture* texture = details::BuildAndFillTexture(
        view, engine, texture_info, std::move(images[i]), out_inflight_creation,
        name);
    if (!texture)
      return Future<absl::Status>(
          absl::InternalError("Failed to create Texture"));
    imp::OwnedTexturePtr owned_texture(
        view.GetTextureFactory().WrapTexture(texture));
    if (texture_info->name()) {
      owned_texture->SetName(texture_info->name()->str());
    }
    out_textures->emplace_back(std::move(owned_texture));
  }

  // Create generic materials.
  std::vector<imp::Future<absl::Status>> combined_futures;
  // Create a map of materials by index to ensure the order is maintained.
  // Note: this is an std::map because of the API around move-only types.
  std::unique_ptr<std::map<uint16_t, GenericMaterialPtr>> materials_by_index =
      std::make_unique<std::map<uint16_t, GenericMaterialPtr>>();

  for (uint16_t material_index = 0; material_index < model->materials()->size();
       ++material_index) {
    const schemas::MaterialInfo* material_schema =
        model->materials()->Get(material_index);

    // The material Create() call is async and TextureBorrower is moved into the
    // lambda, so each material needs its own vector of borrowed textures since
    // this code shouldn't assume how out_textures' lifetime is managed.
    std::vector<BorrowedTexturePtr> textures;
    textures.reserve(out_textures->size());
    absl::c_transform(
        *out_textures, std::back_inserter(textures),
        [](const OwnedTexturePtr& texture) { return texture.Borrow(); });
    // Create a texture borrower that can be used to look up textures by index.
    // Intentionally copy the vector into each provider so that the provider
    // can outlive the original vector and make no assumptions about the Future
    // internals.
    TextureBorrower texture_borrower =
        [textures = std::move(textures)](
            uint64_t texture_index,
            SmallSourceLocation loc) -> BorrowedTexturePtr {
      if (texture_index >= textures.size()) return {};
      return textures[texture_index];
    };
    combined_futures.push_back(
        GenericMaterialImpl::Create(view,
                                    GenericMaterialSpec::FromFlatbuffer(
                                        *material_schema->material()->spec()),
                                    materials_by_params,
                                    material_schema->material()->name()->str())
            .Then([material_index,
                   materials_by_index = materials_by_index.get(),
                   generic_material_parameters =
                       GenericMaterialParameters::FromFlatbuffer(
                           material_schema->material()->params()),
                   texture_borrower = std::move(texture_borrower)](
                      GenericMaterialPtr generic_material) -> absl::Status {
              MP_RETURN_IF_ERROR(generic_material->AssignTexturesAndParams(
                  generic_material_parameters, texture_borrower));
              materials_by_index->emplace(material_index,
                                          std::move(generic_material));
              return absl::OkStatus();
            }));
    // Add the material id to the lookup map.
    material_id_lookup.emplace(material_schema->original_index(),
                               MaterialId{material_index});
  }

  return imp::Future<absl::Status>::CombineList(combined_futures)
      .Then([out_inflight_creation, out_material_configs, out_materials,
             material_count = model->materials()->size(),
             materials_by_index = std::move(materials_by_index)]() {
        // It's critical that we store all the materials in the out_materials
        // vector in the original order. The async GenericMaterialImpl::Create
        // functions may not resolve in the same order as the calls are made,
        // so an intermediate map is used so they are stored by index "key".
        for (int32_t material_index = 0; material_index < material_count;
             ++material_index) {
          GenericMaterialPtr material =
              std::move(materials_by_index->extract(material_index).mapped());
          MaterialConfig material_config(
              material->GetName(), material->GetParameters(),
              material->GetTextures(), material->GetSamplerIndexLookup());
          out_material_configs->push_back(material_config);
          out_materials->push_back(std::move(material));
        }

        out_inflight_creation->FinishPostingResources();
      });
}

OptionalError CreateModelSkeleton(const SkeletonInfo* skeleton,
                                  std::optional<SkeletonData>* out_skeleton) {
  TypedSetVector<BoneData> bones;
  const size_t num_bones = skeleton->parents()->size();
  bones.resize(num_bones);

  if (skeleton->first_children()->size() != num_bones ||
      skeleton->next_siblings()->size() != num_bones ||
      skeleton->local_transforms()->size() != num_bones ||
      skeleton->root_transforms()->size() != num_bones ||
      skeleton->names()->size() != num_bones ||
      skeleton->node_indices()->size() != num_bones) {
    return Error("Invalid Skeleton");
  }

  absl::c_transform(*skeleton->child_counts(),
                    bones.data<BoneData::kNumChildren>(),
                    [](uint16_t child_count) { return child_count; });
  absl::c_transform(*skeleton->parents(), bones.data<BoneData::kParent>(),
                    [](uint16_t parent) { return BoneParentId{parent}; });
  absl::c_transform(
      *skeleton->first_children(), bones.data<BoneData::kFirstChild>(),
      [](uint16_t first_child) { return BoneChildId{first_child}; });
  absl::c_transform(
      *skeleton->next_siblings(), bones.data<BoneData::kNextSibling>(),
      [](uint16_t next_sibling) { return BoneChildId{next_sibling}; });
  absl::c_transform(*skeleton->local_transforms(),
                    bones.data<BoneData::kLocalTransform>(),
                    [](const imp::schemas::PreciseTransform* trs) {
                      return flatbuffers::UnPack(*trs);
                    });
  absl::c_transform(
      *skeleton->root_transforms(), bones.data<BoneData::kRootTransform>(),
      [](const imp::schemas::Mat4* mat) { return flatbuffers::UnPack(*mat); });
  absl::c_transform(
      *skeleton->names(), bones.data<BoneData::kName>(),
      [](const flatbuffers::String* string) { return string->str(); });

  absl::c_transform(*skeleton->node_indices(),
                    bones.data<BoneData::kNodeIndex>(),
                    [](uint16_t index) { return index; });

  MP_RETURN_IF_ERROR(optional_features::VerifyBoneData(bones));

  BoneLookup<HashValue> hashes;
  absl::c_transform(
      *skeleton->names(), std::back_inserter(hashes),
      [](const flatbuffers::String* string) {
        return Hash(absl::string_view(string->c_str(), string->size()));
      });

  BoneLookup<BoneChildId> next_bone_from_hash;
  RobinMap<HashValue, BoneId> first_bone_from_hash;
  RobinMap<HashValue, BoneId> last_bone_from_hash;
  for (BoneId bone : bones.Ids<BoneId>()) {
    HashValue bone_hash = hashes[bone];
    if (auto first_iter = first_bone_from_hash.find(bone_hash);
        first_iter != first_bone_from_hash.end()) {
      if (last_bone_from_hash.empty()) {
        // First duplicate encountered.  Copy the first_bone lookup as a
        // last_bone lookup while this is still the case.
        last_bone_from_hash.insert(first_bone_from_hash.begin(),
                                   first_bone_from_hash.end());
        next_bone_from_hash.resize(bones.size());
      }
      auto last_iter = last_bone_from_hash.find(bone_hash);
      next_bone_from_hash[last_iter->second] = bone;
      last_iter.value() = bone;
    } else {
      first_bone_from_hash.emplace(bone_hash, bone);
      if (!last_bone_from_hash.empty()) {
        last_bone_from_hash.emplace(bone_hash, bone);
      }
    }
  }

  out_skeleton->emplace(SkeletonData{
      std::move(bones), std::move(hashes), std::move(first_bone_from_hash),
      next_bone_from_hash.empty()
          ? std::optional<BoneLookup<BoneChildId>>(std::nullopt)
          : std::optional<BoneLookup<BoneChildId>>(
                std::move(next_bone_from_hash))});

  return NoError();
}

OptionalError CreateModelSkins(
    const flatbuffers::Vector<flatbuffers::Offset<schemas::SkinInfo>>* skins,
    size_t bone_count, TypedVector<ModelData::SkinData>* out_skins,
    absl::flat_hash_map<uint32_t, model::ModelData::SkinId>& skin_id_lookup) {
  for (const schemas::SkinInfo* skin : *skins) {
    TypedVector<ModelData::SampledJointData> sampled_joints;
    absl::c_transform(
        *skin->sampled_joints(), std::back_inserter(sampled_joints),
        [](const schemas::SampledJointInfo* sampled_joint) {
          return SampledJointData{JointId(sampled_joint->joint())};
        });

    SampledJointLookup<filament::math::mat4f> inverse_bind_poses;
    absl::c_transform(*skin->inverse_bind_poses(),
                      std::back_inserter(inverse_bind_poses),
                      [](const imp::schemas::Mat4f* mat) {
                        return flatbuffers::UnPack(*mat);
                      });

    if (sampled_joints.empty() ||
        sampled_joints.size() != inverse_bind_poses.size() ||
        sampled_joints.size() > kMaxValue<uint8_t>) {
      return Error(
          "sampled joints and inverse bind poses out of sync (%d != %d)",
          sampled_joints.size(), inverse_bind_poses.size());
    }
    const auto sampled_joint_count =
        static_cast<int16_t>(sampled_joints.size());

    const Vector<uint16_t>* joint_child_counts = skin->joint_child_counts();
    const Vector<uint16_t>* joint_parents = skin->joint_parents();
    const Vector<uint16_t>* joint_first_children = skin->joint_first_children();
    const Vector<uint16_t>* joint_next_siblings = skin->joint_next_siblings();
    const Vector<uint16_t>* joint_sources = skin->joint_sources();
    const Vector<int16_t>* joint_targets = skin->joint_targets();
    size_t joint_count = skin->joint_child_counts()->size();
    TypedSetVector<JointData> joints;
    joints.reserve(joint_count);

    if (joint_parents->size() != joint_count ||
        joint_first_children->size() != joint_count ||
        joint_next_siblings->size() != joint_count ||
        joint_sources->size() != joint_count ||
        joint_targets->size() != joint_count) {
      return Error("Invalid skin");
    }
    for (size_t joint_index = 0; joint_index < joint_count; ++joint_index) {
      uint16_t source_bone_index = joint_sources->Get(joint_index);
      int16_t sampled_joint_index = joint_targets->Get(joint_index);
      if (source_bone_index >= bone_count || sampled_joint_index < -1 ||
          sampled_joint_index >= sampled_joint_count) {
        return Error("invalid skin");
      }
      joints.push_back(
          joint_child_counts->Get(joint_index),
          ModelData::JointParentId(joint_parents->Get(joint_index)),
          ModelData::JointChildId(joint_first_children->Get(joint_index)),
          ModelData::JointChildId(joint_next_siblings->Get(joint_index)),
          BoneId(source_bone_index), WeakSampledJointId(sampled_joint_index));
    }

    MP_RETURN_IF_ERROR(optional_features::VerifyJointData(joints));

    const Vector<const schemas::SkinnedEntityTargetInfo*>*
        skinned_entity_targets = skin->skinned_entity_targets();
    const Vector<Offset<schemas::SkinnedEntityBoundsInfo>>*
        skinned_entity_joint_bounds = skin->skinned_entity_joint_bounds();
    const Vector<Offset<schemas::SkinnedEntityJointUsageInfo>>*
        skinned_entity_joint_usage = skin->skinned_entity_joint_usage();
    size_t skinned_entity_count = skinned_entity_targets->size();
    TypedSetVector<SkinnedEntityData> skinned_entities;
    skinned_entities.reserve(skinned_entity_count);

    if (skinned_entity_joint_bounds->size() != skinned_entity_count ||
        skinned_entity_joint_usage->size() != skinned_entity_count) {
      return Error("Invalid Skin Item");
    }

    for (size_t skinned_entity_index = 0;
         skinned_entity_index < skinned_entity_count; ++skinned_entity_index) {
      auto target =
          EntityId{skinned_entity_targets->Get(skinned_entity_index)->entity()};
      const schemas::SkinnedEntityBoundsInfo* sampled_joint_bounds_info =
          skinned_entity_joint_bounds->Get(skinned_entity_index);
      SampledJointLookup<filament::Aabb> sampled_joint_bounds;

      sampled_joint_bounds.reserve(sampled_joint_bounds_info->bounds()->size());
      absl::c_transform(*sampled_joint_bounds_info->bounds(),
                        std::back_inserter(sampled_joint_bounds),
                        [](const imp::schemas::Box* box) {
                          float3 center = FromFlatbuffer(box->center());
                          const auto kMinimumHalfExtents = float3(1.0e-3f);
                          float3 half_extent =
                              max(FromFlatbuffer(box->half_extent()),
                                  kMinimumHalfExtents);
                          return filament::Aabb{
                              .min = center - half_extent,
                              .max = center + half_extent,
                          };
                        });

      const schemas::SkinnedEntityJointUsageInfo* sampled_joint_usage =
          skinned_entity_joint_usage->Get(skinned_entity_index);

      auto sampled_joint_in_use = PairedBitVector<SampledJointData>(
          std::vector<uint32_t>(sampled_joint_usage->mask()->begin(),
                                sampled_joint_usage->mask()->end()),
          static_cast<size_t>(sampled_joint_count));
      skinned_entities.push_back(target, std::move(sampled_joint_bounds),
                                 std::move(sampled_joint_in_use));
    }

    auto pose_root = WeakEntityId{skin->pose_root()};

    skin_id_lookup[skin->original_index()] = out_skins->Append(SkinData{
        .sampled_joints = std::move(sampled_joints),
        .inverse_bind_poses = std::move(inverse_bind_poses),
        .joints = std::move(joints),
        .skinned_entities = std::move(skinned_entities),
        .pose_root = pose_root,
    });
  }
  return NoError();
}

}  // namespace

ModelCreator::ModelCreator(filament::Engine* engine,
                           LoaderOptions loader_options)
    : engine_(engine),
      loader_options_(loader_options),
      status_(absl::OkStatus()) {}

ModelCreator::~ModelCreator() {
  // TODO: Somehow, status_ is always ok. Need to make it
  // functional.
  // If we succeeded, there is nothing to do as the ModelData owns any data.
  if (status_.ok()) return;

  for (filament::VertexBuffer* vertex_buffer : vertex_buffers_) {
    engine_->destroy(vertex_buffer);
  }
  for (filament::IndexBuffer* index_buffer : index_buffers_) {
    engine_->destroy(index_buffer);
  }
}

Future<absl::Status> ModelCreator::LoadAll(
    BaseView& view, const schemas::LoadedModel* model,
    MaterialPackage* material_package,
    std::vector<std::unique_ptr<image::ImageContents>> images,
    std::optional<absl::string_view> name) {
  return LoadAllInternal(view, model, material_package, std::move(images),
                         name);
}

Future<absl::Status> ModelCreator::LoadAllInternal(
    BaseView& view, const schemas::LoadedModel* model,
    MaterialPackage* material_package,
    std::vector<std::unique_ptr<image::ImageContents>> images,
    std::optional<absl::string_view> name) {
  IMP_TRACE();
  return CreateModelResources(view, material_package, model, std::move(images),
                              name)
      .Then(
          [this, model]() -> absl::Status {
            IMP_TRACE_BLOCK("Then");
            MP_RETURN_IF_ERROR(CreateModelSkeleton(model->skeleton(), &skeleton_));

            MP_RETURN_IF_ERROR(CreateModelSkins(model->skins(),
                                             skeleton_->bones.size(), &skins_,
                                             skin_id_lookup_));

            lights_punctual_.reserve(model->lights_punctual()->size());
            for (const schemas::LightPunctualInfo* light :
                 *model->lights_punctual()) {
              model::ModelData::LightPunctualData light_data;

              light_data.name = light->name()->str();
              light_data.color =
                  float3{light->color()->r(), light->color()->g(),
                         light->color()->b()};
              light_data.intensity = light->intensity();
              light_data.type = light->type();
              light_data.range = light->range();
              if (light->spot_cone_angles()) {
                light_data.spot_cone_angles =
                    float2{light->spot_cone_angles()->inner(),
                           light->spot_cone_angles()->outer()};
              }
              lights_punctual_.Append(std::move(light_data));
            }

            materials_variants_.reserve(model->materials_variants()->size());
            for (const schemas::MaterialsVariantsInfo* materials_variant :
                 *model->materials_variants()) {
              model::ModelData::MaterialsVariantsData materials_variant_data;
              materials_variant_data.name = materials_variant->name()->str();
              materials_variants_.Append(std::move(materials_variant_data));
            }

            if (model->audio_extension()) {
              audio_emitters_.reserve(
                  model->audio_extension()->emitters()->size());
              for (const schemas::AudioEmitter* emitter :
                   *model->audio_extension()->emitters()) {
                model::ModelData::AudioEmitterData audio_emitter_data;
                audio_emitter_data.name = emitter->name()->str();
                audio_emitter_data.type = emitter->type();
                audio_emitter_data.gain = emitter->gain();
                audio_emitter_data.audio_sources.reserve(
                    emitter->sources()->size());
                for (uint16_t source_index : *emitter->sources()) {
                  audio_emitter_data.audio_sources.push_back(
                      model::ModelData::AudioSourceId(source_index));
                }
                audio_emitters_.Append(std::move(audio_emitter_data));
              }

              audio_sources_.reserve(
                  model->audio_extension()->sources()->size());
              for (const schemas::AudioSource* source :
                   *model->audio_extension()->sources()) {
                model::ModelData::AudioSourceData audio_source_data;
                audio_source_data.name = source->name()->str();
                audio_source_data.audio =
                    model::ModelData::AudioId(source->audio());
                audio_source_data.gain = source->gain();
                audio_source_data.loop = source->loop();
                audio_source_data.auto_play = source->auto_play();
                audio_sources_.Append(std::move(audio_source_data));
              }

              audios_.reserve(model->audio_extension()->audios()->size());
              for (const schemas::Audio* audio :
                   *model->audio_extension()->audios()) {
                model::ModelData::AudioData audio_data;
                absl::string_view buffer(
                    reinterpret_cast<const char*>(audio->buffer()->data()),
                    audio->buffer()->size());
                audio_data.data = absl::Cord(buffer);
                audios_.Append(std::move(audio_data));
              }

              scene_audio_emitters_.reserve(
                  model->audio_extension()->scene_emitters()->size());
              for (const uint16_t emitter_index :
                   *model->audio_extension()->scene_emitters()) {
                scene_audio_emitters_.push_back(
                    model::ModelData::AudioEmitterId(emitter_index));
              }
            }

            // TODO Move this into a separate file to reduce bloat
            if (model->interactivity()) {
              std::unique_ptr<InteractivityModelCreatorExtension>
                  interactivity_extension =
                      extensions::CreateInteractivityModelCreatorExtension();
              if (interactivity_extension) {
                absl::StatusOr<model::ModelData::InteractivityData>
                    interactivity_result =
                        interactivity_extension->DeserializeInteractivityData(
                            model->interactivity());
                if (!interactivity_result.ok()) {
                  return interactivity_result.status();
                }
                interactivity_ = *std::move(interactivity_result);
              } else {
                return Error(
                    "Failed to create InteractivityMediaCreatorExtension.");
              }
            }

            skinning_buffers_.reserve(model->skinning_buffers()->size());
            for (const schemas::SkinningBufferInfo* skinning_buffer :
                 *model->skinning_buffers()) {
              if (skinning_buffer->bone_indices_and_weights()->size() == 0) {
                return Error("Invalid skinning buffer size.");
              }
              model::ModelData::SkinningBufferData skinning_buffer_data;
              skinning_buffer_data.bone_indices_and_weights.resize(
                  skinning_buffer->bone_indices_and_weights()->size());
              absl::c_transform(
                  *skinning_buffer->bone_indices_and_weights(),
                  skinning_buffer_data.bone_indices_and_weights.data(),
                  [](const imp::schemas::Float2* vec) {
                    return flatbuffers::UnPack(*vec);
                  });
              skinning_buffers_.push_back(std::move(skinning_buffer_data));
            }

            MP_RETURN_IF_ERROR(CreateModelEntityGraph(
                engine_, model, model->entity_graph(), &entities_));

            MP_RETURN_IF_ERROR(optional_features::VerifyEntityData(entities_));

            return absl::OkStatus();
          });
}

absl::Status ModelCreator::TryComplete() {
  return inflight_creation_.TryComplete();
}

bool ModelCreator::IsFullyLoaded() {
  return inflight_creation_.IsFullyLoaded();
}

bool ModelCreator::HasPendingWork() {
  return inflight_creation_.HasPendingWork();
}

void ModelCreator::WhenFullyLoaded(std::function<void()> cb) {
  inflight_creation_.WhenFullyLoaded(std::move(cb));
}

void ModelCreator::RemoveWhenFullyLoadedCallback() {
  inflight_creation_.RemoveWhenFullyLoadedCallback();
}

Future<absl::Status> ModelCreator::CreateModelResources(
    BaseView& view, MaterialPackage* material_package,
    const schemas::LoadedModel* model,
    std::vector<std::unique_ptr<image::ImageContents>> images,
    std::optional<absl::string_view> name) {
  IMP_TRACE();
  // Immediately start unzipping raw material data on a background thread.
  absl::flat_hash_set<GenericMaterialSpec> requested_materials;
  for (const schemas::MaterialInfo* material_info : *model->materials()) {
    requested_materials.insert(GenericMaterialSpec::FromFlatbuffer(
        *material_info->material()->spec()));
  }

  // Loader already has the correct material package.
  return material_package
      ->GetOrLoadMaterials(view, engine_, requested_materials)
      .Then(
          [this, &view, model, name = std::optional<std::string>(name),
           images = std::move(images)](const MaterialPackage::MaterialCache&
                                           materials_by_params) mutable {
            IMP_TRACE_BLOCK("Then");
            return imp::loader::details::CreateModelResources(
                view, engine_, loader_options_, model, std::move(images),
                &vertex_buffers_, &index_buffers_, &morph_target_buffers_,
                &morph_target_uv0_textures_, &textures_, &materials_,
                &stored_vertex_data_, &stored_index_data_,
                &material_config_info_, &inflight_creation_,
                materials_by_params, material_id_lookup_, name);
          });
}

absl::StatusOr<std::unique_ptr<model::ModelData>> ModelCreator::CreateModelData(
    imp::BaseView* view) {
  IMP_TRACE();
  if (!status_.ok()) {
    return status_;
  }
  // This will move the contents of all our vectors into the ModelData, so even
  // though our status is still ok, we won't delete any resources in the
  // destructor.

  auto result = absl::WrapUnique(new ModelData(
      view, engine_, std::move(entities_), std::move(skins_),
      std::move(lights_punctual_), std::move(materials_variants_),
      std::move(*skeleton_), std::move(vertex_buffers_),
      std::move(index_buffers_), std::move(morph_target_buffers_),
      std::move(morph_target_uv0_textures_), std::move(textures_),
      std::move(materials_), std::move(material_id_lookup_),
      std::move(skin_id_lookup_), std::move(skinning_buffers_),
      std::move(stored_vertex_data_), std::move(stored_index_data_),
      std::move(material_config_info_), std::move(audio_emitters_),
      std::move(audio_sources_), std::move(audios_),
      std::move(scene_audio_emitters_), std::move(interactivity_)));

  return result;
}

}  // namespace imp::loader::details
