/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
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
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_animator.h"
#include "apibindings/base_asset_loader.h"
#include "core/math/math.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Manages glTF model loading, instantiation, animation, and properties for the
// Jetpack XR Scene.
// TODO: Add unit tests for this class.
class ModelManager {
 public:
  virtual ~ModelManager() = default;

  // Loads the asset pointer of a glTF model from the local assets folder or
  // a remote URL, and resolves the asset loader when it is ready.
  virtual void LoadGltfAsset(absl::string_view path,
                             std::unique_ptr<BaseAssetLoader> asset_loader) = 0;

  // Loads the asset pointer of an glTF model from a byte array, and returns a
  // unique identifier for it when it is ready.
  virtual void LoadGltfAsset(absl::Cord data, absl::string_view key,
                             std::unique_ptr<BaseAssetLoader> asset_loader) = 0;

  // Releases the asset pointer of previously loaded glTF model if the reference
  // count is 0, otherwise decrements the reference count.
  virtual absl::Status ReleaseGltfAsset(std::intptr_t gltf_token) = 0;

  // Instantiates a glTF model and returns an entity ID.
  virtual absl::StatusOr<int32_t> InstanceGltfModel(std::intptr_t gltf_token,
                                                    bool enable_collider) = 0;

  // Attaches or detaches a collider to a glTF model.
  virtual absl::Status SetGltfModelColliderEnabled(int32_t node,
                                                   bool enable_collider) = 0;

  // Attaches or detaches a footprint affordance to a glTF model.
  virtual absl::Status SetGltfReformAffordanceEnabled(
      int32_t impress_node, bool enable_affordance,
      bool system_movable = true) = 0;

  // Animates a glTF model.
  virtual void AnimateGltfModel(
      int32_t node, absl::string_view animation_name, bool loop, float speed,
      float start_time, int32_t channel_id,
      std::unique_ptr<BaseAssetAnimator> asset_animator) = 0;

  // Stops the animation of a glTF model.
  virtual absl::Status StopGltfModelAnimation(int32_t node,
                                              int32_t channel_id) = 0;

  // Pause or resume the animation of a glTF model. If `toggle` = true resume
  // the animation, `toggle` = false pause the animation.
  virtual absl::Status ToggleGltfModelAnimation(int32_t node, bool toggle,
                                                int32_t channel_id) = 0;

  // Sets the speed of the animation of a glTF model.
  virtual absl::Status SetGltfModelAnimationSpeed(int32_t node, float speed,
                                                  int32_t channel_id) = 0;

  // Sets the playback time of the animation of a glTF model.
  virtual absl::Status SetGltfModelAnimationPlaybackTime(
      int32_t node, float playback_time, int32_t channel_id) = 0;

  // Gets the total count of the animation clips in the glTF model.
  virtual absl::StatusOr<int32_t> GetGltfModelAnimationCount(int32_t node) = 0;

  // Gets the name of an animation clip by index.
  virtual absl::StatusOr<std::string> GetGltfModelAnimationName(
      int32_t node, int32_t index) = 0;

  // Gets the duration of an animation clip by index.
  virtual absl::StatusOr<float> GetGltfModelAnimationDurationSeconds(
      int32_t node, int32_t index) = 0;

  // Returns the local space unscaled bounds of the glTF model.
  virtual absl::StatusOr<imp::Box> GetGltfModelLocalBounds(int32_t node) = 0;

  // Sets the material override directly on a specific node's mesh at a given
  // primitive index.
  virtual absl::Status SetGltfModelNodeMaterialOverride(
      int32_t node_id, std::intptr_t material, size_t primitive_index) = 0;

  // Clears the material override directly on a specific node's mesh at a given
  // primitive index.
  virtual absl::Status ClearGltfModelNodeMaterialOverride(
      int32_t node_id, size_t primitive_index) = 0;

  // Schedules reskinning for a glTF model using its entity ID.
  virtual absl::Status ScheduleReskinning(int32_t node_id) = 0;

  // Called by ImpressApiView::Update() to manage animation callbacks.
  virtual void Update(const FrameTime& frame_time) = 0;

  // Resets the animation contexts for cleanup purposes.
  virtual void ResetAnimationContexts() = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_MODEL_MANAGER_H_
