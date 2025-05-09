// Copyright 2024 Google LLC
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

#include "core/loader/ipc/ipc_test_helpers.h"

#include <vector>

#include "devtools/build/runtime/get_runfiles_dir.h"
#include "core/common/file_helpers.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::ipc {

std::string GetSampleAssetsDirectory() {
  const auto sample_assets_directory =
      devtools_build::testonly::GetTestSrcdir() +
      "/google3/third_party/arcore/googledata/sceneform/assets/";
  return sample_assets_directory;
}

OptionalError LoadAsset(std::string friendly_path, BufferAccess* out_access) {
  auto path = GetSampleAssetsDirectory() + friendly_path;
  MP_RETURN_IF_ERROR(LoadBinary(path, out_access));
  return NoError();
}

OptionalError LoadAsset(std::string friendly_path,
                        std::vector<uint8_t>* out_data) {
  BufferAccess access;
  MP_RETURN_IF_ERROR(LoadAsset(friendly_path, &access));

  const uint8_t* access_data = reinterpret_cast<const uint8_t*>(access.Data());
  *out_data = std::vector<uint8_t>(access_data, access_data + access.Size());
  return NoError();
}

}  // namespace imp::loader::ipc
