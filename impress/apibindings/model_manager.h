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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_MODEL_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_MODEL_MANAGER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/asset_animator.h"
#include "apibindings/asset_loader.h"
#include "core/math/math.h"
#include "core/ncsb/component_handle.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/utils/frame_time.h"

namespace imp {

class ImpressApiView;

// Manages glTF model loading, instantiation, animation, and properties for the
// Jetpack XR Scene.
// TODO: Add unit tests for this class.
class ModelManager {
 public:
  explicit ModelManager(ImpressApiView& view);
  virtual ~ModelManager() = default;

  // Loads the asset pointer of a glTF model from the local assets folder or
  // a remote URL, and resolves the asset loader when it is ready.
  virtual void LoadGltfAsset(absl::string_view path,
                             std::unique_ptr<AssetLoader> asset_loader);
  // Loads the asset pointer of a glTF model from a absl::Cord.
  virtual void LoadGltfAsset(absl::Cord data, absl::string_view key,
                             std::unique_ptr<AssetLoader> asset_loader);
  // Releases the asset pointer of previously loaded glTF asset.
  virtual absl::Status ReleaseGltfAsset(std::intptr_t gltf_token);
  // Instantiates a glTF model and returns an entity ID.
  virtual absl::StatusOr<int32_t> InstanceGltfModel(std::intptr_t gltf_token,
                                                    bool enable_collider);
  // Attaches or detaches a collider to a glTF model.
  virtual absl::Status SetGltfModelColliderEnabled(int32_t node,
                                                   bool enable_collider);
  // Animates a glTF model.
  virtual void AnimateGltfModel(int32_t node, absl::string_view animation_name,
                                bool loop,
                                std::unique_ptr<AssetAnimator> asset_animator);
  // Stops the animation of a glTF model.
  virtual absl::Status StopGltfModelAnimation(int32_t node);
  // Returns the local space unscaled bounds of the glTF model.
  virtual absl::StatusOr<imp::Box> GetGltfModelLocalBounds(int32_t node);

  // Sets the material override for a node's mesh at a given primitive index.
  virtual absl::Status SetMaterialOverride(int32_t node_id,
                                           std::intptr_t material,
                                           absl::string_view node_name,
                                           size_t primitive_index);

  // Clears the material override for a node's mesh at a given primitive index.
  virtual absl::Status ClearMaterialOverride(int32_t node_id,
                                             absl::string_view node_name,
                                             size_t primitive_index);

  // Called by ImpressApiView::Update() to manage animation callbacks.
  void Update(const FrameTime& frame_time);

  // Disposes all glTF assets and instances.
  void DisposeGltfAssetsAndInstances();

 private:
  // Returns a glTF mesh component for a subnode of an instance of a glTF asset
  // in the Jepack XR scene.
  absl::StatusOr<ComponentHandle<GltfMesh>> FindGltfMeshByNodeName(
      int32_t node_id, absl::string_view node_name);

 protected:
  ImpressApiView& view_;

 private:
  absl::flat_hash_map<int32_t,
                      std::tuple<ComponentHandle<GltfAnimator>,
                                 std::optional<std::unique_ptr<AssetAnimator>>>>
      node_to_anim_ctx_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_MODEL_MANAGER_H_
