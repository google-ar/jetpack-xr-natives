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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERER_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/media/media_color_space.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/display_color_space.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/renderer_policy_handler.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer_context.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// The SplitEngineRenderer is responsible for creating and managing the
// schema-defined 3D content for a given app (identified by its BridgeId).
class SplitEngineRenderer {
 public:
  virtual ~SplitEngineRenderer() = default;

  using OnFinishedCallback = Invocable<void()>;

  // Handles a Command schema message from the app with the given bridge id.
  // The verifier is used to validate the command.
  // The on_finished_callback is invoked when SplitEngineRenderer is completely
  // finished processing the command, including any asynchronous GPU uploads
  // of the data buffers contained within `command`.
  // The caller must ensure memory backing the message must remain alive until
  // the on_finished_callback is invoked.
  virtual absl::Status HandleCommand(
      BridgeId bridge_id, flatbuffers::Verifier& verifier,
      const android_xr::schemas::Command& command,
      OnFinishedCallback on_finished) = 0;
  // Handles an android_xr::schemas::Request sent from the app, for example to
  // create a built-in material.
  virtual Future<absl::Status> HandleRequest(
      BridgeId bridge_id, const android_xr::schemas::Request& request) = 0;

  virtual absl::Status AddMeshData(
      const android_xr::schemas::AddMeshData& command,
      OnFinishedCallback on_finished) = 0;
  virtual absl::Status RemoveMeshData(
      const android_xr::schemas::RemoveMeshData& command) = 0;

  virtual absl::Status AddMorphTargetBuffers(
      const android_xr::schemas::AddMorphTargetBuffers& command) = 0;
  virtual absl::Status RemoveMorphTargetBuffers(
      const android_xr::schemas::RemoveMorphTargetBuffers& command) = 0;

  virtual absl::Status AddTextures(
      const android_xr::schemas::AddTextures& command,
      OnFinishedCallback on_finished) = 0;
  virtual absl::Status RemoveTextures(
      const android_xr::schemas::RemoveTextures& command) = 0;

  virtual absl::Status AddMaterials(
      const android_xr::schemas::AddMaterials& command) = 0;
  virtual absl::Status RemoveMaterials(
      const android_xr::schemas::RemoveMaterials& command) = 0;
  virtual absl::Status AddMaterialInstances(
      const android_xr::schemas::AddMaterialInstances& command) = 0;
  virtual absl::Status DuplicateMaterialInstances(
      const android_xr::schemas::DuplicateMaterialInstances& command) = 0;
  virtual absl::Status RemoveMaterialInstances(
      const android_xr::schemas::RemoveMaterialInstances& command) = 0;

  virtual absl::Status AddNodes(
      const android_xr::schemas::AddNodes& command) = 0;
  virtual absl::Status UpdateNodes(
      const android_xr::schemas::UpdateNodes& command, BridgeId bridge_id) = 0;
  virtual absl::Status RemoveNodes(
      const android_xr::schemas::RemoveNodes& command) = 0;
  virtual absl::Status AssignUserIdToNodes(
      const android_xr::schemas::AssignUserIdToNodes& command) = 0;

  virtual absl::Status AddRenderables(
      const android_xr::schemas::AddRenderables& command) = 0;
  virtual absl::Status UpdateRenderables(
      const android_xr::schemas::UpdateRenderables& command) = 0;
  virtual absl::Status RemoveRenderables(
      const android_xr::schemas::RemoveRenderables& command) = 0;

  // Adds the collider if the same type of the collider doesn't exist in the
  // node, otherwise updates the collider.
  virtual absl::Status AddOrUpdateColliders(
      const android_xr::schemas::AddOrUpdateColliders& command) = 0;
  // Removes the collider if the node has the collider type, otherwise error.
  virtual absl::Status RemoveColliders(
      const android_xr::schemas::RemoveColliders& command) = 0;

  virtual absl::Status SetMaterialParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::SetMaterialParameters& command) = 0;
  virtual absl::Status SetBuiltInMaterialParameters(
      flatbuffers::Verifier& verifier,
      const android_xr::schemas::SetBuiltInMaterialParameters& command) = 0;

  virtual Future<absl::Status> CreateBuiltInMaterial(
      BridgeId bridge_id,
      const android_xr::schemas::BuiltInMaterialRequest& request) = 0;
  virtual Future<std::vector<BuiltInMaterialPtr>>
  PreloadBuiltInCustomMaterials() = 0;
  virtual Future<absl::Status> CreateCustomMaterial(
      BridgeId bridge_id,
      const android_xr::schemas::AddCustomMaterialRequest& request) = 0;

  virtual absl::Status AddImageBasedLightingAssets(
      const android_xr::schemas::AddImageBasedLightingAssets& command) = 0;
  virtual absl::Status RemoveImageBasedLightingAssets(
      const android_xr::schemas::RemoveImageBasedLightingAssets& command) = 0;
  virtual absl::Status SetPreferredEnvironmentIblAsset(
      const android_xr::schemas::SetPreferredEnvironmentIblAsset& command) = 0;
  virtual absl::StatusOr<int32_t> GetImageBasedLightingAssetCount(
      BridgeId bridge_id) = 0;

  // Sets an external texture for the given bridge id.
  //
  // The content_security_level is the security level of the content being
  // rendered, used for DRM support.
  //
  // get_source_color_space_fn is used to get the source texture color space.
  //
  // The texture should be removed normally using RemoveTextures with the same
  // TextureId, at which point the release_fn will be invoked.
  virtual void SetTextureExternal(
      BridgeId bridge_id, TextureId texture_id, BorrowedTexturePtr texture,
      std::function<MediaColorSpace()> get_source_color_space_fn,
      std::function<void*()> get_surface_fn,
      imp::Invocable<void()> release_fn) = 0;
  // Returns the maximum content security level among all external textures.
  // If bridge_id is provided, only the content security level for the given
  // bridge id will be considered.
  virtual ContentSecurityLevel GetMaximumContentSecurityLevel(
      std::optional<BridgeId> bridge_id) = 0;
  // Returns the display color space required by textures in the renderer.
  // If bridge_id is provided, only the color space requirements for the given
  // bridge id will be considered.
  virtual DisplayColorSpace GetRequiredDisplayColorSpace(
      std::optional<BridgeId> bridge_id) = 0;

  // Iterates over all active surfaces and invoke the provided function.
  // If bridge_id is provided, only the surfaces for the given bridge id will
  // be considered.
  virtual void ForEachActiveSurface(std::optional<BridgeId> bridge_id,
                                    std::function<void(void*)> fn) = 0;

  // Overrides the channel for all renderables associated with the given
  // user_id.
  virtual absl::Status SetChannelOverride(uint64_t user_id,
                                          uint8_t channel) = 0;
  // Clears any channel override set for the given user_id, restoring the
  // original channel.
  virtual absl::Status ClearChannelOverride(uint64_t user_id) = 0;

  // Adds permission grants for the application, identified with its BridgeId.
  virtual void AddAppPermission(BridgeId bridge_id,
                                AppPermission app_permission) = 0;

  // Removes permission grants for the application, identified with its
  // BridgeId.
  virtual void RemoveAppPermission(BridgeId bridge_id,
                                   AppPermission app_permission) = 0;

  // Sets the RendererPolicyHandler for a given application context.
  virtual void SetRendererPolicyHandler(
      BridgeId bridge_id,
      std::unique_ptr<RendererPolicyHandler> renderer_policy_handler) = 0;

  // Returns the AppContext for a given bridge id, or nullptr if not found.
  virtual const AppContext* /*absl_nullable*/  GetAppContext(
      BridgeId bridge_id) const = 0;
  // Returns the AppContext for a given bridge id, or nullptr if not found.
  virtual AppContext* /*absl_nullable*/  GetAppContext(BridgeId bridge_id) = 0;
  // Sets the current context for all updates to the app with this bridge id.
  virtual void SetAppContext(BridgeId bridge_id) = 0;
  // Destroys all Nodes, materials, etc. associated with the given bridge id.
  virtual absl::Status ClearAppContext(BridgeId bridge_id) = 0;

  // Used by Split Engine renderer to determine whether to allow the given API
  // level. The API level comes from a Split Engine flatbuffer table that the
  // renderer is deciding whether to process. If incoming messages have
  // disallowed API levels, the renderer will reject the message from the app
  // which sent the disallowed request, along with every subsequent API calls.
  // A special value of kExperimentalApiLevel means that all API levels are
  // allowed.
  // The validation is inclusive of the range [1, api_level].
  virtual void SetValidationApiLevel(int32_t api_level) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_RENDERER_H_
