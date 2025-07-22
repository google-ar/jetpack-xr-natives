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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/buffer_access.h"
#include "core/loader/provider/extensions/gltf_extension_meshopt.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"

namespace imp::loader::extensions {

absl::StatusOr<BufferAccess> ResolveMeshOpt(imp::gltf::imp_proto::Gltf* gltf) {
  for (auto& buffer_view : gltf->buffer_views) {
    if (buffer_view.extensions.meshopt_compression) {
      return absl::InternalError(
          "Meshopt extension is not supported in this build!");
    }
  }

  return BufferAccess{};
}

}  // namespace imp::loader::extensions
