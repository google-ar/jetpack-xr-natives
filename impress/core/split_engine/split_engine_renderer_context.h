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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERER_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERER_CONTEXT_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "core/assets/asset_cache.h"
#include "core/common/invocable.h"
#include "core/common/rememberer.h"
#include "core/common/robin_map.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/materials/material.h"
#include "core/media/media_color_space.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/ncsb/node_handle.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/renderer_policy_handler.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_filament_resource_ptrs.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

// Holds all EnvironmentLights for a single app.
struct EnvironmentLightContext {
  // An asset cache of ImageBasedLightingAssets.
  AssetCache<ImageBasedLightingAsset> image_based_lighting_assets;

  // A map from front end image based lighting assed ids to
  // a corresponding EnvironmentLightPtr.
  absl::flat_hash_map<uint64_t, OwnedEnvironmentLightPtr> lights;

  // A set of image based lighting asset ids that have been requested to be
  // removed. The actual removal happens in the Update() function only if
  // all borrows are released. This allows an external system to use the IBL
  // assets (i.e. to do environment transitions) beyond the lifetime of the
  // Split Engine application.
  std::vector<OwnedEnvironmentLightPtr> pending_removes;

  // Whether the app context associated with this environment light context
  // has been destroyed.
  bool app_context_destroyed = false;
};

// The application permission controller serves to grant application
// permission, remove application permissions and control permissions.
class AppPermissionController {
 public:
  explicit AppPermissionController(BaseView& view);
  ~AppPermissionController() = default;
  void AddPermissions(AppPermission permission);
  void RemovePermissions(AppPermission permission);
  bool HasPermission(AppPermissionTypes permission) const;

  // Update the application current state based on the state of the
  // controller.
  void Update();

  // Checks node to validate against current state of the controller.
  void ApplyControls(imp::NodeHandle node);

  // Returns the default parent node for all nodes in this application.
  imp::NodeHandle GetDefaultParentNode();

 private:
  BaseView& view_;
  // EnumFlag containing all app permissions
  AppPermission app_permissions_;
  // Default parent node for all nodes in this application
  // The node is parent of all incoming nodes, unless it is reparented.
  // The node tied to user_id will eventually be reparented.
  imp::NodeHandle default_parent_node_;
};

struct VertexBufferData {
  OwnedVertexBufferPtr vertex_buffer;
  MeshVertexDataPtr vertex_data;
};

struct IndexBufferData {
  OwnedIndexBufferPtr index_buffer;
  MeshIndexDataPtr index_data;
};

// Holds all Nodes, materials, textures, meshes, etc. for a single app.
// The app context inherits from Rememberer so that it can be used to cancel
// futures when the app context is destroyed.
struct AppContext : public Rememberer {
  // TODO: (broken link) - eliminate the need for std::pair usage here.
  using RawMaterialInstance =
      std::pair<BorrowedFilamentMaterialPtr, OwnedMaterialPtr>;
  // A raw material instance or a built-in material instance.
  using RawOrBuiltInMaterialInstance =
      std::variant<RawMaterialInstance, BuiltInMaterialPtr>;

  AppContext(BaseView& view, EnvironmentLightContext& environment_light_cxt,
             BridgeId bridge_id);
  ~AppContext();

  BaseView& view;

  // A map from front end entity ids to the corresponding NodeHandle.
  RobinMap<uint32_t, NodeHandle> entity_map;

  // A map from user id to the corresponding NodeHandle.
  RobinMap<uint64_t, NodeHandle> user_id_to_node;

  // A map from front end material ids to the corresponding
  // filament::Material.
  //
  // Note: the filament::Material* will be nullptr if the material is a
  // built-in material. In that case, it is simply used as a placeholder to
  // map the material id to the built-in material instance.
  //
  // The uint64_t id comes from the memory address of the front end
  // material.
  RobinMap<uint64_t, OwnedFilamentMaterialPtr> materials;

  // A map from front end material instance ids to the corresponding renderer
  // material instance. This can either be a placeholder material or a
  // built-in material. In local mode, the placeholder material will be a real
  // filament material instance created from a raw binary material.
  //
  // The uint64_t id comes from the memory address of the front end material
  // instance.
  RobinMap<uint64_t, RawOrBuiltInMaterialInstance> material_instances;

  // A map from front end vertex buffer ids to the corresponding
  // VertexBufferData.
  //
  // The uint64_t id comes from the memory address of the front end vertex
  // buffer.
  RobinMap<uint64_t, VertexBufferData> vertex_buffers;

  // A map from front end index buffer ids to the corresponding
  // IndexBufferData.
  //
  // The uint64_t id comes from the memory address of the front end index
  // buffer.
  RobinMap<uint64_t, IndexBufferData> index_buffers;

  // A map from front end morph target buffer ids to the corresponding
  // OwnedMorphTargetBufferPtr.
  //
  // The uint64_t id comes from the memory address of the front end morph
  // target buffer.
  RobinMap<uint64_t, OwnedMorphTargetBufferPtr> morph_target_buffers;

  // A map from front end texture ids to the corresponding
  // filament::Texture.
  //
  // The texture id comes from the memory address of the front end texture.
  RobinMap<TextureId, OwnedOrBorrowedTexturePtr> textures;

  struct TextureExternalMetadata {
    // Used to release the texture when it is destroyed.
    imp::Invocable<void()> release_fn;
    // A function that returns the source color space for this texture.
    //
    // Having this as a function since the color space can't be determined
    // until the first frame is decoded. Since we are checking the color space
    // only when the texture is in use, we can use the function to get the
    // correct color space.
    std::function<MediaColorSpace()> get_source_color_space_fn;

    // Returns a weak reference to the surface for this texture as void*.
    std::function<void*()> get_surface_fn;
  };
  // A map from front end texture ids to a corresponding release function
  // for the external textures. Used to release the texture when it is
  // destroyed.
  //
  // The texture id comes from the memory address of the front end texture.
  RobinMap<TextureId, TextureExternalMetadata> textures_external;

  // The EnvironmentLightContext not owned by the AppContext - it has
  // a separate map so that it can have a longer lifetime than the
  // AppContext.
  EnvironmentLightContext& environment_light_context;

  // Holder for the application permissions and controls.
  AppPermissionController app_permission_controller;

  // The RendererPolicyHandler is optional and not owned by the
  // AppContext. If present, the system will handle updates for
  // this context.
  std::unique_ptr<RendererPolicyHandler> renderer_policy_handler = nullptr;

  // The bridge id that the app this context is associated with.
  BridgeId bridge_id = UINT64_MAX;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERER_CONTEXT_H_
