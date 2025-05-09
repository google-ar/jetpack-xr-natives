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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_ASSET_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "core/animation/gltf_animation.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/collision/collision_accelerator_provider.h"
#include "core/collision/mesh_collision_accelerator.h"
#include "core/common/paired_vector.h"
#include "core/common/robin_map.h"
#include "core/common/typed_id.h"
#include "core/common/typed_vector.h"
#include "core/material_library/material_param_value.h"
#include "core/math/mat.h"
#include "core/model/model_data.h"
#include "core/model/shared_data.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/string_map.h"

namespace imp {

using GenericMaterialListing =
    PairedVector<GenericMaterialPtr, GenericMaterialPtr>;

class GltfAssetLoader;

class GltfAsset {
 public:
  // Event fired when a GltfAsset either loads successfully or fails.
  // If the asset is retrieved from the AssetManager but it was cached from a
  // previous load, this event is not sent.
  struct LoadEvent : public Event {
    // The identifier for the gltf asset.
    AssetId asset_id;
    // kOk if the load was successfully, otherwise, the load failed. Will be
    // kCancelled if the load was cancelled.
    absl::Status status;

    // Timestamps for when the total load began and ended.
    absl::Time start_time = absl::InfinitePast();
    absl::Time end_time = absl::InfinitePast();

    // Timestamps for when the material & model downloads began and ended.
    //
    // Note: The material & model start downloading at the same time.
    // Use GetTotalDownloadDuration to get the total time spent downloading.
    absl::Time start_download_materials_and_model_time = absl::InfinitePast();
    absl::Time end_download_materials_time = absl::InfinitePast();
    absl::Time end_download_model_time = absl::InfinitePast();

    // Timestamps for when downloading dependent files in the model (i.e. remote
    // textures) began and ended.
    // This happens after the materials and model are downloaded.
    // Use GetTotalDownloadDuration to get the total time spent downloading.
    absl::Time start_download_deps_time = absl::InfinitePast();
    absl::Time end_download_deps_time = absl::InfinitePast();

    // Timestamps for parsing the gltf asset.
    // This happens after the materials and model are downloaded.
    // Downloading missing deps happens *during* this stage.
    // Use GetTotalParseTime() to get the total time spent parsing excluding
    // downloads.
    absl::Time start_parse_time = absl::InfinitePast();
    absl::Time end_parse_time = absl::InfinitePast();

    // total bytes downloaded, which is essentially the material bytes + the
    // model bytes. This includes bytes retrieved from the http cache.
    size_t num_bytes_downloaded = 0;

    // Returns the total time spent on the gltf asset load.
    // Will return -absl::InfiniteDuration() if data isn't populated.
    absl::Duration GetTotalDuration() const;

    // Returns the total time spent downloading for the gltf asset load.
    // This includes materials, models, and dependencies within the model.
    // Will return -absl::InfiniteDuration() if data isn't populated.
    absl::Duration GetTotalDownloadDuration() const;

    // Returns the total time spent parsing the downloaded resources of the gltf
    // asset.
    // Will return -absl::InfiniteDuration() if data isn't populated.
    absl::Duration GetTotalParseDuration() const;
  };

  using GltfAnimDataPtr = std::unique_ptr<animation::GltfAnimation>;
  using AnimId = TypedId<GltfAnimDataPtr, int32_t>;
  template <typename T>
  using AnimLookup = PairedVector<T, GltfAnimDataPtr>;
  using AnimNamesSpan = absl::Span<const std::string>;
  using AnimNamesList = std::vector<std::string>;
  using MeshCollisionAcceleratorPool =
      RobinMap<model::EntityId, std::unique_ptr<MeshCollisionAccelerator>>;

  // Accessibility to vertex attributes.
  // LINT.IfChange(VertexAccessFlags)
  enum VertexAccessFlags : uint8_t {
    kNone = 0,
    kPosition = 1 << 0,
    kTangent = 1 << 1,
    // TODO Add support for skinning and morph target information.
    kDefault = kNone
  };
  // LINT.ThenChange(
  //     //depot/google3/third_party/impress/core/loader/loader_options.h:VertexAccessFlags,
  //     //depot/google3/third_party/split_engine/schemas/split_engine_data.fbs:VertexAccessFlags
  // )

  struct LoadOptions {
    // Specifies which (if any) vertex attributes to continue storing after
    // uploading data to the GPU so that it is accessible via the GltfAsset.
    uint8_t vertex_access_flags = kDefault;
    // If true, opt-in to the generic 'lite' materials (no clearcoat/occlusion).
    bool use_lite_materials = false;
    // If true, then the glTF will not include some nodes that don't impact the
    // visual rendering of the glTF. For instance, nodes in the middle of the
    // hierarchy that don't contain a mesh and aren't using in animation.
    bool exclude_excess_nodes = false;
    // If true, all baked shadow planes in the glTF will be removed. This is
    // useful for applications that wish to render their own implementation of
    // shadows on the glTFs.
    bool remove_shadow_planes = false;
    // An optional URL override for the default materials.  Only applicable
    // before the first gltf asset load is attempted.  Used for unit tests that
    // cannot hit gstatic.
    absl::optional<std::string> materials_url_override;

    // Determines which (if any) collider components will be automatically added
    // to the nodes in the glTF scene.
    GltfState::ColliderMode collider_mode =
        GltfState::ColliderMode::GLTF_COLLIDER_BOUNDS_PER_MESH_DEFAULT;

    // Determines whether GltfRenderer will use the shared material instance
    // directly or duplicate it and use the duplicated instance.
    GltfState::MaterialSharingMode material_sharing_mode =
        GltfState::MaterialSharingMode::DUPLICATED_DEFAULT;

    // Specifies (if any) a list of transforms for instancing of the glTF, which
    // will be local to the transform of the node the GltfRenderer is on. The
    // default behavior would be an empty list to not enable instancing.
    std::vector<mat4f> instance_transforms = {};

    // The FutureGroup into which all futures will be added.
    absl::optional<FutureGroup> future_group = absl::nullopt;
  };

  static const MaterialPreCompileOptions& kDefaultMaterialPreCompileOptions;

  class Builder {
   public:
    explicit Builder(size_t animation_count) noexcept;

    Builder(Builder const& rhs) = delete;
    Builder(Builder&& rhs) noexcept;
    ~Builder() noexcept;
    Builder& operator=(Builder& rhs) = delete;
    Builder& operator=(Builder&& rhs) noexcept;

    Builder& Model(std::unique_ptr<model::ModelData> model_data) noexcept;
    Builder& Animation(size_t animation_index, absl::string_view name,
                       GltfAnimDataPtr animation) noexcept;
    Builder& SharedMaterials(GenericMaterialListing shared_materials) noexcept;

    absl::StatusOr<std::unique_ptr<GltfAsset>> Build();

   private:
    std::unique_ptr<model::ModelData> model_data_;
    TypedVector<GltfAnimDataPtr> animations_;
    AnimLookup<std::string> animation_names_;
    StringMap<AnimId> animation_name_lookup_;
    GenericMaterialListing shared_materials_;
  };

  static Future<std::unique_ptr<GltfAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future, GltfAssetLoader* loader,
      GltfAsset::LoadOptions options);

  GltfAsset();

  // GltfAsset is not copyable.
  GltfAsset(const GltfAsset&) = delete;
  GltfAsset& operator=(const GltfAsset& rhs) = delete;

  // GltfAsset is movable.
  GltfAsset(GltfAsset&& rhs) = default;
  GltfAsset& operator=(GltfAsset&& rhs) = default;

  const model::ModelData& GetModelData() const;
  const MeshCollisionAccelerator* GetMeshCollisionAccelerator(
      model::ModelData::EntityId entity_id) const;

  AnimNamesSpan GetAnimNames() const;
  AnimId GetAnimId(absl::string_view anim_name) const;
  const animation::GltfAnimation* GetGltfAnimData(AnimId anim_id) const;
  size_t AnimationCount() const;

  const GenericMaterialListing& GetSharedMaterials() const;

  void BuildMeshCollisionAccelerators(
      CollisionAcceleratorProvider* collision_accelerator_provider);

 private:
  GltfAsset(std::unique_ptr<model::ModelData> model_data,
            TypedVector<GltfAnimDataPtr>&& animations,
            AnimLookup<std::string>&& animation_names,
            StringMap<AnimId>&& animation_name_lookup,
            GenericMaterialListing&& shared_materials);

  std::unique_ptr<model::ModelData> model_data_;
  TypedVector<GltfAnimDataPtr> animations_;
  AnimLookup<std::string> animation_names_;
  StringMap<AnimId> animation_name_lookup_;
  GenericMaterialListing shared_materials_;
  MeshCollisionAcceleratorPool mesh_collision_accelerators_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_ASSET_H_
