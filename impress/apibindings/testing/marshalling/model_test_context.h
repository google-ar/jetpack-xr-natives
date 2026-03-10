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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_MODEL_TEST_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_MODEL_TEST_CONTEXT_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

namespace imp {

// Context that models the state of the model tests.
class ModelTestContext {
 public:
  static ModelTestContext& Get() {
    static ModelTestContext instance;
    return instance;
  }

  static constexpr int64_t kUninitialized64 = 0L;
  static constexpr int32_t kUninitialized32 = 0;
  static constexpr size_t kUninitializedSize = 0;

  struct LoadGltfAssetPathParams {
    std::string expected_path;
    int64_t success_token = kUninitialized64;
    std::string failure_message;
    std::string actual_path;

    void Reset() {
      expected_path.clear();
      success_token = kUninitialized64;
      failure_message.clear();
      actual_path.clear();
    }
  };

  struct LoadGltfAssetBytesParams {
    bool expect_test_pattern = false;
    size_t expected_size = kUninitializedSize;
    std::string expected_key;
    std::string actual_key;
    int64_t success_token = kUninitialized64;
    std::string failure_message;

    void Reset() {
      expect_test_pattern = false;
      expected_size = kUninitializedSize;
      expected_key.clear();
      actual_key.clear();
      success_token = kUninitialized64;
      failure_message.clear();
    }
  };

  struct ReleaseGltfAssetParams {
    int64_t expected_token = kUninitialized64;
    int64_t actual_token = kUninitialized64;

    void Reset() {
      expected_token = kUninitialized64;
      actual_token = kUninitialized64;
    }
  };

  struct InstanceGltfModelParams {
    int64_t expected_token = kUninitialized64;
    bool expected_collider = false;
    int32_t success_id = kUninitialized32;
    int64_t actual_token = kUninitialized64;
    bool actual_collider = false;

    void Reset() {
      expected_token = kUninitialized64;
      expected_collider = false;
      success_id = kUninitialized32;
      actual_token = kUninitialized64;
      actual_collider = false;
    }
  };

  struct SetGltfModelColliderEnabledParams {
    int32_t expected_node_id = kUninitialized32;
    bool expected_enabled = false;
    int32_t actual_node_id = kUninitialized32;
    bool actual_enabled = false;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_enabled = false;
      actual_node_id = kUninitialized32;
      actual_enabled = false;
    }
  };

  struct SetGltfReformAffordanceEnabledParams {
    int32_t expected_node_id = kUninitialized32;
    bool expected_enabled = false;
    bool expected_system_movable = false;
    int32_t actual_node_id = kUninitialized32;
    bool actual_enabled = false;
    bool actual_system_movable = false;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_enabled = false;
      expected_system_movable = false;
      actual_node_id = kUninitialized32;
      actual_enabled = false;
      actual_system_movable = false;
    }
  };

  struct AnimateGltfModelParams {
    int32_t expected_node_id = kUninitialized32;
    std::string expected_name;
    bool expected_loop = false;
    int32_t expected_channel_id = kUninitialized32;
    float expected_speed = 0.0f;
    float expected_start_time = 0.0f;
    std::string failure_message;
    int32_t actual_node_id = kUninitialized32;
    std::string actual_name;
    bool actual_loop = false;
    int32_t actual_channel_id = kUninitialized32;
    float actual_speed = 0.0f;
    float actual_start_time = 0.0f;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_name.clear();
      expected_loop = false;
      expected_channel_id = kUninitialized32;
      expected_speed = 0.0f;
      expected_start_time = 0.0f;
      failure_message.clear();
      actual_node_id = kUninitialized32;
      actual_name.clear();
      actual_loop = false;
      actual_channel_id = kUninitialized32;
      actual_speed = 0.0f;
      actual_start_time = 0.0f;
    }
  };

  struct StopGltfModelAnimationParams {
    int32_t expected_node_id = kUninitialized32;
    int32_t expected_channel_id = kUninitialized32;
    int32_t actual_node_id = kUninitialized32;
    int32_t actual_channel_id = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_channel_id = kUninitialized32;
      actual_node_id = kUninitialized32;
      actual_channel_id = kUninitialized32;
    }
  };

  struct ToggleGltfModelAnimationParams {
    int32_t expected_node_id = kUninitialized32;
    bool expected_toggle = false;
    int32_t expected_channel_id = kUninitialized32;
    int32_t actual_node_id = kUninitialized32;
    bool actual_toggle = false;
    int32_t actual_channel_id = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_toggle = false;
      expected_channel_id = kUninitialized32;
      actual_node_id = kUninitialized32;
      actual_toggle = false;
      actual_channel_id = kUninitialized32;
    }
  };

  struct SetGltfModelAnimationSpeedParams {
    int32_t expected_node_id = kUninitialized32;
    float expected_speed = 0.0f;
    int32_t expected_channel_id = kUninitialized32;
    int32_t actual_node_id = kUninitialized32;
    float actual_speed = 0.0f;
    int32_t actual_channel_id = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_speed = 0.0f;
      expected_channel_id = kUninitialized32;
      actual_node_id = kUninitialized32;
      actual_speed = 0.0f;
      actual_channel_id = kUninitialized32;
    }
  };

  struct SetGltfModelAnimationPlaybackTimeParams {
    int32_t expected_node_id = kUninitialized32;
    float expected_playback_time = 0.0f;
    int32_t expected_channel_id = kUninitialized32;
    int32_t actual_node_id = kUninitialized32;
    float actual_playback_time = 0.0f;
    int32_t actual_channel_id = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_playback_time = 0.0f;
      expected_channel_id = kUninitialized32;
      actual_node_id = kUninitialized32;
      actual_playback_time = 0.0f;
      actual_channel_id = kUninitialized32;
    }
  };

  struct GetGltfModelAnimationCountParams {
    int32_t expected_node_id = kUninitialized32;
    int32_t success_count = kUninitialized32;
    int32_t actual_node_id = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      success_count = kUninitialized32;
      actual_node_id = kUninitialized32;
    }
  };

  struct GetGltfModelAnimationNameParams {
    int32_t expected_node_id = kUninitialized32;
    int32_t expected_index = kUninitialized32;
    std::string success_name;
    int32_t actual_node_id = kUninitialized32;
    int32_t actual_index = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_index = kUninitialized32;
      success_name.clear();
      actual_node_id = kUninitialized32;
      actual_index = kUninitialized32;
    }
  };

  struct GetGltfModelLocalBoundsParams {
    int32_t expected_node_id = kUninitialized32;
    float success_center[3] = {0.0f, 0.0f, 0.0f};
    float success_half_extent[3] = {0.0f, 0.0f, 0.0f};
    int32_t actual_node_id = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      std::fill_n(success_center, 3, 0.0f);
      std::fill_n(success_half_extent, 3, 0.0f);
      actual_node_id = kUninitialized32;
    }
  };

  struct SetMaterialOverrideParams {
    int32_t expected_node_id = kUninitialized32;
    int64_t expected_material_handle = kUninitialized64;
    std::string expected_node_name;
    int32_t expected_primitive_index = kUninitialized32;
    int32_t actual_node_id = kUninitialized32;
    int64_t actual_material_handle = kUninitialized64;
    std::string actual_node_name;
    int32_t actual_primitive_index = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_material_handle = kUninitialized64;
      expected_node_name.clear();
      expected_primitive_index = kUninitialized32;
      actual_node_id = kUninitialized32;
      actual_material_handle = kUninitialized64;
      actual_node_name.clear();
      actual_primitive_index = kUninitialized32;
    }
  };

  struct ClearMaterialOverrideParams {
    int32_t expected_node_id = kUninitialized32;
    std::string expected_node_name;
    int32_t expected_primitive_index = kUninitialized32;
    int32_t actual_node_id = kUninitialized32;
    std::string actual_node_name;
    int32_t actual_primitive_index = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_node_name.clear();
      expected_primitive_index = kUninitialized32;
      actual_node_id = kUninitialized32;
      actual_node_name.clear();
      actual_primitive_index = kUninitialized32;
    }
  };

  struct GetGltfModelAnimationDurationSecondsParams {
    int32_t expected_node_id = kUninitialized32;
    int32_t expected_index = kUninitialized32;
    float success_duration = 0.0f;
    int32_t actual_node_id = kUninitialized32;
    int32_t actual_index = kUninitialized32;

    void Reset() {
      expected_node_id = kUninitialized32;
      expected_index = kUninitialized32;
      success_duration = 0.0f;
      actual_node_id = kUninitialized32;
      actual_index = kUninitialized32;
    }
  };

  void Reset() {
    load_gltf_asset_path.Reset();
    load_gltf_asset_bytes.Reset();
    release_gltf_asset.Reset();
    instance_gltf_model.Reset();
    set_gltf_model_collider_enabled.Reset();
    set_gltf_reform_affordance_enabled.Reset();
    animate_gltf_model.Reset();
    stop_gltf_model_animation.Reset();
    toggle_gltf_model_animation.Reset();
    set_gltf_model_animation_speed.Reset();
    set_gltf_model_animation_playback_time.Reset();
    get_gltf_model_animation_count.Reset();
    get_gltf_model_animation_name.Reset();
    get_gltf_model_animation_duration_seconds.Reset();
    get_gltf_model_local_bounds.Reset();
    set_material_override.Reset();
    clear_material_override.Reset();
  }

  LoadGltfAssetPathParams load_gltf_asset_path;
  LoadGltfAssetBytesParams load_gltf_asset_bytes;
  ReleaseGltfAssetParams release_gltf_asset;
  InstanceGltfModelParams instance_gltf_model;
  SetGltfModelColliderEnabledParams set_gltf_model_collider_enabled;
  SetGltfReformAffordanceEnabledParams set_gltf_reform_affordance_enabled;
  AnimateGltfModelParams animate_gltf_model;
  StopGltfModelAnimationParams stop_gltf_model_animation;
  ToggleGltfModelAnimationParams toggle_gltf_model_animation;
  SetGltfModelAnimationSpeedParams set_gltf_model_animation_speed;
  SetGltfModelAnimationPlaybackTimeParams
      set_gltf_model_animation_playback_time;
  GetGltfModelAnimationCountParams get_gltf_model_animation_count;
  GetGltfModelAnimationNameParams get_gltf_model_animation_name;
  GetGltfModelAnimationDurationSecondsParams
      get_gltf_model_animation_duration_seconds;
  GetGltfModelLocalBoundsParams get_gltf_model_local_bounds;
  SetMaterialOverrideParams set_material_override;
  ClearMaterialOverrideParams clear_material_override;

 private:
  ModelTestContext() = default;
  ~ModelTestContext() = default;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_MODEL_TEST_CONTEXT_H_
