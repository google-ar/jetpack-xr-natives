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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_IMPRESS_API_TEST_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_IMPRESS_API_TEST_CONTEXT_H_

#include <algorithm>
#include <cstdint>
#include <string>

namespace imp {

// Context that models the test state of the Impress API.
class ImpressApiTestContext {
 public:
  static ImpressApiTestContext& Get() {
    static ImpressApiTestContext instance;
    return instance;
  }

  void Reset() {
    // LoadGltfAsset (path)
    expected_gltf_path.clear();
    gltf_asset_loader_success_token = 0L;
    gltf_asset_loader_failure_message.clear();
    actual_gltf_path.clear();

    // LoadGltfAsset (byte array)
    expected_gltf_data.clear();
    expected_gltf_key.clear();
    actual_gltf_data.clear();
    actual_gltf_key.clear();

    // ReleaseGltfAsset
    expected_gltf_token_release = 0L;
    actual_gltf_token_release = 0L;

    // InstanceGltfModel
    expected_gltf_token_instance = 0L;
    expected_instance_collider = false;
    instance_gltf_model_success_id = 0;
    actual_gltf_token_instance = 0L;
    actual_instance_collider = false;

    // SetGltfModelColliderEnabled
    expected_node_id_collider = 0;
    expected_collider_enabled = false;
    actual_node_id_collider = 0;
    actual_collider_enabled = false;

    // AnimateGltfModel
    expected_node_id_anim = 0;
    expected_anim_name.clear();
    expected_anim_loop = false;
    animator_failure_message.clear();
    actual_node_id_anim = 0;
    actual_anim_name.clear();
    actual_anim_loop = false;

    // StopGltfModelAnimation
    expected_node_id_stop_anim = 0;
    actual_node_id_stop_anim = 0;

    // GetGltfModelLocalBounds
    expected_node_id_bounds = 0;
    std::fill_n(bounds_success_center, 3, 0.0f);
    std::fill_n(bounds_success_half_extent, 3, 0.0f);
    actual_node_id_bounds = 0;

    // SetMaterialOverride
    expected_node_id_set_override = 0;
    expected_material_handle = 0L;
    expected_node_name_set_override.clear();
    expected_primitive_index_set_override = 0;
    actual_node_id_set_override = 0;
    actual_material_handle = 0L;
    actual_node_name_set_override.clear();
    actual_primitive_index_set_override = 0;

    // ClearMaterialOverride
    expected_node_id_clear_override = 0;
    expected_node_name_clear_override.clear();
    expected_primitive_index_clear_override = 0;
    actual_node_id_clear_override = 0;
    actual_node_name_clear_override.clear();
    actual_primitive_index_clear_override = 0;
  }

  // LoadGltfAsset (path)
  std::string expected_gltf_path;
  int64_t gltf_asset_loader_success_token = 0L;
  std::string gltf_asset_loader_failure_message;
  std::string actual_gltf_path;

  // LoadGltfAsset (byte array)
  std::string expected_gltf_data;
  std::string expected_gltf_key;
  std::string actual_gltf_data;
  std::string actual_gltf_key;

  // ReleaseGltfAsset
  int64_t expected_gltf_token_release = 0L;
  int64_t actual_gltf_token_release = 0L;

  // InstanceGltfModel
  int64_t expected_gltf_token_instance = 0L;
  bool expected_instance_collider = false;
  int32_t instance_gltf_model_success_id = 0;
  int64_t actual_gltf_token_instance = 0L;
  bool actual_instance_collider = false;

  // SetGltfModelColliderEnabled
  int32_t expected_node_id_collider = 0;
  bool expected_collider_enabled = false;
  int32_t actual_node_id_collider = 0;
  bool actual_collider_enabled = false;

  // AnimateGltfModel
  int32_t expected_node_id_anim = 0;
  std::string expected_anim_name;
  bool expected_anim_loop = false;
  std::string animator_failure_message;
  int32_t actual_node_id_anim = 0;
  std::string actual_anim_name;
  bool actual_anim_loop = false;

  // StopGltfModelAnimation
  int32_t expected_node_id_stop_anim = 0;
  int32_t actual_node_id_stop_anim = 0;

  // GetGltfModelLocalBounds
  int32_t expected_node_id_bounds = 0;
  float bounds_success_center[3];
  float bounds_success_half_extent[3];
  int32_t actual_node_id_bounds = 0;

  // SetMaterialOverride
  int32_t expected_node_id_set_override = 0;
  int64_t expected_material_handle = 0L;
  std::string expected_node_name_set_override;
  int32_t expected_primitive_index_set_override = 0;
  int32_t actual_node_id_set_override = 0;
  int64_t actual_material_handle = 0L;
  std::string actual_node_name_set_override;
  int32_t actual_primitive_index_set_override = 0;

  // ClearMaterialOverride
  int32_t expected_node_id_clear_override = 0;
  std::string expected_node_name_clear_override;
  int32_t expected_primitive_index_clear_override = 0;
  int32_t actual_node_id_clear_override = 0;
  std::string actual_node_name_clear_override;
  int32_t actual_primitive_index_clear_override = 0;

 private:
  ImpressApiTestContext() = default;
  ~ImpressApiTestContext() = default;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_IMPRESS_API_TEST_CONTEXT_H_
