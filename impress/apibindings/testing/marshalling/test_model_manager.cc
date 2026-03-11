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

#include "apibindings/testing/marshalling/test_model_manager.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "gmock/gmock.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_animator.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/testing/marshalling/jni_test_utils.h"
#include "apibindings/testing/marshalling/model_test_context.h"
#include "core/geometry/shapes/box.h"
#include "core/view/utils/frame_time.h"

namespace imp {

TestModelManager::TestModelManager(ImpressApiView& view) {}

void TestModelManager::LoadGltfAsset(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
  ModelTestContext& context = ModelTestContext::Get();
  context.load_gltf_asset_path.actual_path = std::string(path);

  

  if (asset_loader != nullptr) {
    if (!context.load_gltf_asset_path.failure_message.empty()) {
      asset_loader->OnFailure(context.load_gltf_asset_path.failure_message);
    } else {
      asset_loader->OnSuccess(context.load_gltf_asset_path.success_token);
    }
  }
}

void TestModelManager::LoadGltfAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  ModelTestContext& context = ModelTestContext::Get();

  context.load_gltf_asset_bytes.actual_key = std::string(key);
  
  if (context.load_gltf_asset_bytes.expect_test_pattern) {
    EXPECT_OK(
        VerifyTestPattern(data, context.load_gltf_asset_bytes.expected_size));
  }

  if (asset_loader != nullptr) {
    if (!context.load_gltf_asset_bytes.failure_message.empty()) {
      asset_loader->OnFailure(context.load_gltf_asset_bytes.failure_message);
    } else {
      asset_loader->OnSuccess(context.load_gltf_asset_bytes.success_token);
    }
  }
}

absl::Status TestModelManager::ReleaseGltfAsset(std::intptr_t gltf_token) {
  ModelTestContext& context = ModelTestContext::Get();
  context.release_gltf_asset.actual_token = gltf_token;

  

  return absl::OkStatus();
}

absl::StatusOr<int32_t> TestModelManager::InstanceGltfModel(
    std::intptr_t gltf_token, bool enable_collider) {
  ModelTestContext& context = ModelTestContext::Get();
  context.instance_gltf_model.actual_token = gltf_token;
  context.instance_gltf_model.actual_collider = enable_collider;

  
  

  return context.instance_gltf_model.success_id;
}

absl::Status TestModelManager::SetGltfModelColliderEnabled(
    int32_t node, bool enable_collider) {
  ModelTestContext& context = ModelTestContext::Get();
  context.set_gltf_model_collider_enabled.actual_node_id = node;
  context.set_gltf_model_collider_enabled.actual_enabled = enable_collider;

  
  

  return absl::OkStatus();
}

absl::Status TestModelManager::SetGltfReformAffordanceEnabled(
    int32_t impress_node, bool enable_affordance) {
  ModelTestContext& context = ModelTestContext::Get();
  context.set_gltf_reform_affordance_enabled.actual_node_id = impress_node;
  context.set_gltf_reform_affordance_enabled.actual_enabled = enable_affordance;

  
  

  return absl::OkStatus();
}

void TestModelManager::AnimateGltfModel(
    int32_t node, absl::string_view animation_name, bool loop,
    std::unique_ptr<BaseAssetAnimator> asset_animator) {
  ModelTestContext& context = ModelTestContext::Get();
  context.animate_gltf_model.actual_node_id = node;
  context.animate_gltf_model.actual_name = std::string(animation_name);
  context.animate_gltf_model.actual_loop = loop;

  
  
  

  if (asset_animator != nullptr) {
    if (!context.animate_gltf_model.failure_message.empty()) {
      asset_animator->OnFailure(context.animate_gltf_model.failure_message);
    } else {
      asset_animator->OnComplete();
    }
  }
}

absl::Status TestModelManager::StopGltfModelAnimation(int32_t node) {
  ModelTestContext& context = ModelTestContext::Get();
  context.stop_gltf_model_animation.actual_node_id = node;
  
  return absl::OkStatus();
}

absl::Status TestModelManager::ToggleGltfModelAnimation(int32_t node,
                                                        bool toggle) {
  return absl::UnimplementedError(
      "TestModelManager::ToggleGltfModelAnimation unimplemented");
}

absl::StatusOr<imp::Box> TestModelManager::GetGltfModelLocalBounds(
    int32_t node) {
  ModelTestContext& context = ModelTestContext::Get();
  context.get_gltf_model_local_bounds.actual_node_id = node;

  

  imp::Box mock_box;
  mock_box.center[0] = context.get_gltf_model_local_bounds.success_center[0];
  mock_box.center[1] = context.get_gltf_model_local_bounds.success_center[1];
  mock_box.center[2] = context.get_gltf_model_local_bounds.success_center[2];
  mock_box.halfExtent[0] =
      context.get_gltf_model_local_bounds.success_half_extent[0];
  mock_box.halfExtent[1] =
      context.get_gltf_model_local_bounds.success_half_extent[1];
  mock_box.halfExtent[2] =
      context.get_gltf_model_local_bounds.success_half_extent[2];

  return mock_box;
}

absl::Status TestModelManager::SetMaterialOverride(int32_t node_id,
                                                   std::intptr_t material,
                                                   absl::string_view node_name,
                                                   size_t primitive_index) {
  ModelTestContext& context = ModelTestContext::Get();
  context.set_material_override.actual_node_id = node_id;
  context.set_material_override.actual_material_handle = material;
  context.set_material_override.actual_node_name = std::string(node_name);
  context.set_material_override.actual_primitive_index = primitive_index;

  
  
  
  

  return absl::OkStatus();
}

absl::Status TestModelManager::ClearMaterialOverride(
    int32_t node_id, absl::string_view node_name, size_t primitive_index) {
  ModelTestContext& context = ModelTestContext::Get();
  context.clear_material_override.actual_node_id = node_id;
  context.clear_material_override.actual_node_name = std::string(node_name);
  context.clear_material_override.actual_primitive_index = primitive_index;

  
  
  

  return absl::OkStatus();
}

void TestModelManager::Update(const FrameTime& frame_time) {
  IMP_LOG(imp::FATAL) << "TestModelManager::Update unimplemented";
}

void TestModelManager::ResetAnimationContexts() {
  IMP_LOG(imp::FATAL) << "TestModelManager::ResetAnimationContexts unimplemented";
}

}  // namespace imp
