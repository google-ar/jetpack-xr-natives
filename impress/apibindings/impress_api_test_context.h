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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_IMPRESS_API_TEST_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_IMPRESS_API_TEST_CONTEXT_H_

#include <cstdint>
#include <string>

namespace imp {

class ImpressApiTestContext {
 public:
  static ImpressApiTestContext& Get() {
    static ImpressApiTestContext instance;
    return instance;
  }

  void Reset() {
    expected_gltf_path.clear();
    gltf_asset_loader_success_token = 0L;
    gltf_asset_loader_failure_message.clear();
    actual_gltf_path.clear();
  }

  std::string expected_gltf_path;
  int64_t gltf_asset_loader_success_token = 0L;
  std::string gltf_asset_loader_failure_message;
  std::string actual_gltf_path;

 private:
  ImpressApiTestContext() = default;
  ~ImpressApiTestContext() = default;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_IMPRESS_API_TEST_CONTEXT_H_
