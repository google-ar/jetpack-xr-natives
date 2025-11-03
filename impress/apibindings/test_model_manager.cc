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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/asset_animator.h"
#include "apibindings/asset_loader.h"
#include "apibindings/impress_api_test_context.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/model_manager.h"
#include "core/geometry/shapes/box.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Test implementation for ModelManager.
ModelManager::ModelManager(ImpressApiView& view) : view_(view) {}

void ModelManager::LoadGltfAsset(absl::string_view path,
                                 std::unique_ptr<AssetLoader> asset_loader) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_gltf_path = std::string(path);
  

  if (asset_loader != nullptr) {
    if (!context.gltf_asset_loader_failure_message.empty()) {
      asset_loader->OnFailure(context.gltf_asset_loader_failure_message);
    } else {
      asset_loader->OnSuccess(context.gltf_asset_loader_success_token);
    }
  }
}

void ModelManager::LoadGltfAsset(absl::Cord data, absl::string_view key,
                                 std::unique_ptr<AssetLoader> asset_loader) {
  IMP_LOG(imp::ERROR) << "TestModelManager::LoadGltfAsset(Cord) unimplemented.";
}

absl::Status ModelManager::ReleaseGltfAsset(std::intptr_t gltf_token) {
  return absl::UnimplementedError("TestModelManager::ReleaseGltfAsset");
}

absl::StatusOr<int32_t> ModelManager::InstanceGltfModel(
    std::intptr_t gltf_token, bool enable_collider) {
  return absl::UnimplementedError("TestModelManager::InstanceGltfModel");
}

absl::Status ModelManager::SetGltfModelColliderEnabled(int32_t node,
                                                       bool enable_collider) {
  return absl::UnimplementedError(
      "TestModelManager::SetGltfModelColliderEnabled");
}

void ModelManager::AnimateGltfModel(
    int32_t node, absl::string_view animation_name, bool loop,
    std::unique_ptr<AssetAnimator> asset_animator) {
  IMP_LOG(imp::ERROR) << "TestModelManager::AnimateGltfModel unimplemented.";
}

absl::Status ModelManager::StopGltfModelAnimation(int32_t node) {
  return absl::UnimplementedError("TestModelManager::StopGltfModelAnimation");
}

absl::StatusOr<imp::Box> ModelManager::GetGltfModelLocalBounds(int32_t node) {
  return absl::UnimplementedError("TestModelManager::GetGltfModelLocalBounds");
}

absl::Status ModelManager::SetMaterialOverride(int32_t node_id,
                                               std::intptr_t material,
                                               absl::string_view node_name,
                                               size_t primitive_index) {
  return absl::UnimplementedError("TestModelManager::SetMaterialOverride");
}

absl::Status ModelManager::ClearMaterialOverride(int32_t node_id,
                                                 absl::string_view node_name,
                                                 size_t primitive_index) {
  return absl::UnimplementedError("TestModelManager::ClearMaterialOverride");
}

void ModelManager::Update(const FrameTime& frame_time) {
  // No-op in test implementation.
}

void ModelManager::DisposeGltfAssetsAndInstances() {
  // No-op in test implementation.
}

}  // namespace imp
