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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEXTURE_TEST_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEXTURE_TEST_CONTEXT_H_

#include <cstdint>
#include <string>

namespace imp {

// Context that models the state of the texture tests.
class TextureTestContext {
 public:
  static TextureTestContext& Get() {
    static TextureTestContext instance;
    return instance;
  }

  static constexpr int64_t kUninitialized64 = 0L;

  struct LoadTextureAssetPathParams {
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

  struct BorrowReflectionTextureParams {
    bool expected_call = false;
    bool actual_call = false;
    int64_t success_token = kUninitialized64;

    void Reset() {
      expected_call = false;
      actual_call = false;
      success_token = kUninitialized64;
    }
  };

  struct GetReflectionTextureFromIblParams {
    int64_t expected_ibl_token = kUninitialized64;
    int64_t actual_ibl_token = kUninitialized64;
    int64_t success_token = kUninitialized64;

    void Reset() {
      expected_ibl_token = kUninitialized64;
      actual_ibl_token = kUninitialized64;
      success_token = kUninitialized64;
    }
  };

  struct BorrowTextureParams {
    int64_t expected_handle = kUninitialized64;
    int64_t actual_handle = kUninitialized64;

    void Reset() {
      expected_handle = kUninitialized64;
      actual_handle = kUninitialized64;
    }
  };

  void Reset() {
    load_texture_asset_path.Reset();
    borrow_reflection_texture.Reset();
    get_reflection_texture_from_ibl.Reset();
    borrow_texture.Reset();
  }

  LoadTextureAssetPathParams load_texture_asset_path;
  BorrowReflectionTextureParams borrow_reflection_texture;
  GetReflectionTextureFromIblParams get_reflection_texture_from_ibl;
  BorrowTextureParams borrow_texture;

 private:
  TextureTestContext() = default;
  ~TextureTestContext() = default;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_TEXTURE_TEST_CONTEXT_H_
