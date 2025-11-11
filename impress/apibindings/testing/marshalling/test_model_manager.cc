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

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "apibindings/base_asset_animator.h"
#include "apibindings/base_asset_loader.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/testing/marshalling/impress_api_test_context.h"
#include "core/geometry/shapes/box.h"
#include "core/view/utils/frame_time.h"

namespace imp {

TestModelManager::TestModelManager(ImpressApiView& view) {}

void TestModelManager::LoadGltfAsset(
    absl::string_view path, std::unique_ptr<BaseAssetLoader> asset_loader) {
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

void TestModelManager::LoadGltfAsset(
    absl::Cord data, absl::string_view key,
    std::unique_ptr<BaseAssetLoader> asset_loader) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_gltf_data = std::string(data);
  context.actual_gltf_key = std::string(key);

  
  

  if (asset_loader != nullptr) {
    if (!context.gltf_asset_loader_failure_message.empty()) {
      asset_loader->OnFailure(context.gltf_asset_loader_failure_message);
    } else {
      asset_loader->OnSuccess(context.gltf_asset_loader_success_token);
    }
  }
}

absl::Status TestModelManager::ReleaseGltfAsset(std::intptr_t gltf_token) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_gltf_token_release = gltf_token;

  

  return absl::OkStatus();
}

absl::StatusOr<int32_t> TestModelManager::InstanceGltfModel(
    std::intptr_t gltf_token, bool enable_collider) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_gltf_token_instance = gltf_token;
  context.actual_instance_collider = enable_collider;

  
  

  return context.instance_gltf_model_success_id;
}

absl::Status TestModelManager::SetGltfModelColliderEnabled(
    int32_t node, bool enable_collider) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_node_id_collider = node;
  context.actual_collider_enabled = enable_collider;

  
  

  return absl::OkStatus();
}

void TestModelManager::AnimateGltfModel(
    int32_t node, absl::string_view animation_name, bool loop,
    std::unique_ptr<BaseAssetAnimator> asset_animator) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_node_id_anim = node;
  context.actual_anim_name = std::string(animation_name);
  context.actual_anim_loop = loop;

  
  
  

  if (asset_animator != nullptr) {
    if (!context.animator_failure_message.empty()) {
      asset_animator->OnFailure(context.animator_failure_message);
    } else {
      asset_animator->OnComplete();
    }
  }
}

absl::Status TestModelManager::StopGltfModelAnimation(int32_t node) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_node_id_stop_anim = node;
  
  return absl::OkStatus();
}

absl::Status TestModelManager::ToggleGltfModelAnimation(int32_t node,
                                                        bool toggle) {
  return absl::UnimplementedError(
      "TestModelManager::ToggleGltfModelAnimation unimplemented");
}

absl::StatusOr<imp::Box> TestModelManager::GetGltfModelLocalBounds(
    int32_t node) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_node_id_bounds = node;

  

  imp::Box mock_box;
  mock_box.center[0] = context.bounds_success_center[0];
  mock_box.center[1] = context.bounds_success_center[1];
  mock_box.center[2] = context.bounds_success_center[2];
  mock_box.halfExtent[0] = context.bounds_success_half_extent[0];
  mock_box.halfExtent[1] = context.bounds_success_half_extent[1];
  mock_box.halfExtent[2] = context.bounds_success_half_extent[2];

  return mock_box;
}

absl::Status TestModelManager::SetMaterialOverride(int32_t node_id,
                                                   std::intptr_t material,
                                                   absl::string_view node_name,
                                                   size_t primitive_index) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_node_id_set_override = node_id;
  context.actual_material_handle = material;
  context.actual_node_name_set_override = std::string(node_name);
  context.actual_primitive_index_set_override = primitive_index;

  
  
  
  

  return absl::OkStatus();
}

absl::Status TestModelManager::ClearMaterialOverride(
    int32_t node_id, absl::string_view node_name, size_t primitive_index) {
  ImpressApiTestContext& context = ImpressApiTestContext::Get();
  context.actual_node_id_clear_override = node_id;
  context.actual_node_name_clear_override = std::string(node_name);
  context.actual_primitive_index_clear_override = primitive_index;

  
  
  

  return absl::OkStatus();
}

void TestModelManager::Update(const FrameTime& frame_time) {
  IMP_LOG(imp::FATAL) << "TestModelManager::Update unimplemented";
}

void TestModelManager::ResetAnimationContexts() {
  IMP_LOG(imp::FATAL) << "TestModelManager::ResetAnimationContexts unimplemented";
}

}  // namespace imp
