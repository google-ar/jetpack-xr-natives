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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_MODEL_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_MODEL_MANAGER_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_animator.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/model_manager.h"
#include "core/geometry/shapes/box.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Inherits from the real ModelManager for testing purposes.
class TestModelManager : public ModelManager {
 public:
  explicit TestModelManager(ImpressApiView& view);
  ~TestModelManager() override = default;

  void LoadGltfAsset(absl::string_view path,
                     std::unique_ptr<BaseAssetLoader> asset_loader) override;
  void LoadGltfAsset(absl::Cord data, absl::string_view key,
                     std::unique_ptr<BaseAssetLoader> asset_loader) override;
  absl::Status ReleaseGltfAsset(std::intptr_t gltf_token) override;
  absl::StatusOr<int32_t> InstanceGltfModel(std::intptr_t gltf_token,
                                            bool enable_collider) override;
  absl::Status SetGltfModelColliderEnabled(int32_t node,
                                           bool enable_collider) override;
  absl::Status SetGltfReformAffordanceEnabled(int32_t impress_node,
                                              bool enable_affordance) override;
  void AnimateGltfModel(
      int32_t node, absl::string_view animation_name, bool loop,
      std::unique_ptr<BaseAssetAnimator> asset_animator) override;
  absl::Status StopGltfModelAnimation(int32_t node) override;
  absl::Status ToggleGltfModelAnimation(int32_t node, bool toggle) override;
  absl::StatusOr<imp::Box> GetGltfModelLocalBounds(int32_t node) override;
  absl::Status SetMaterialOverride(int32_t node_id, std::intptr_t material,
                                   absl::string_view node_name,
                                   size_t primitive_index) override;
  absl::Status ClearMaterialOverride(int32_t node_id,
                                     absl::string_view node_name,
                                     size_t primitive_index) override;
  void Update(const FrameTime& frame_time) override;
  void ResetAnimationContexts() override;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEST_MODEL_MANAGER_H_
