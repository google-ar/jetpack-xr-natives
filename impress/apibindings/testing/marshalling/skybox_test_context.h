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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_SKYBOX_TEST_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_SKYBOX_TEST_CONTEXT_H_

#include <cstddef>
#include <cstdint>
#include <string>

namespace imp {

// Context that models the state of the skybox tests.
class SkyboxTestContext {
 public:
  static SkyboxTestContext& Get() {
    static SkyboxTestContext instance;
    return instance;
  }

  static constexpr int64_t kUninitialized64 = 0L;
  static constexpr size_t kUninitializedSize = 0;

  struct LoadImageBasedLightingAssetPathParams {
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

  struct LoadImageBasedLightingAssetBytesParams {
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

  struct ReleaseImageBasedLightingAssetParams {
    int64_t expected_token = kUninitialized64;
    int64_t actual_token = kUninitialized64;

    void Reset() {
      expected_token = kUninitialized64;
      actual_token = kUninitialized64;
    }
  };

  struct SetEnvironmentLightParams {
    int64_t expected_token = kUninitialized64;
    int64_t actual_token = kUninitialized64;

    void Reset() {
      expected_token = kUninitialized64;
      actual_token = kUninitialized64;
    }
  };

  struct ClearEnvironmentLightParams {
    bool expected_clear = false;
    bool actual_clear = false;

    void Reset() {
      expected_clear = false;
      actual_clear = false;
    }
  };

  void Reset() {
    load_image_based_lighting_asset_path.Reset();
    load_image_based_lighting_asset_bytes.Reset();
    release_image_based_lighting_asset.Reset();
    set_environment_light.Reset();
    clear_environment_light.Reset();
  }

  LoadImageBasedLightingAssetPathParams load_image_based_lighting_asset_path;
  LoadImageBasedLightingAssetBytesParams load_image_based_lighting_asset_bytes;
  ReleaseImageBasedLightingAssetParams release_image_based_lighting_asset;
  SetEnvironmentLightParams set_environment_light;
  ClearEnvironmentLightParams clear_environment_light;

 private:
  SkyboxTestContext() = default;
  ~SkyboxTestContext() = default;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_SKYBOX_TEST_CONTEXT_H_
