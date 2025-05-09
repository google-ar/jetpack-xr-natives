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

#include <vector>

#include "core/common/optional_error.h"
#include "core/loader/provider/extensions/gltf_extension_draco.h"

namespace imp {
namespace loader {
namespace extensions {

absl::StatusOr<std::vector<BufferAccess>> ResolveDraco(imp::gltf::Gltf* gltf) {
  for (auto& mesh : gltf->meshes) {
    for (auto& prim : mesh.primitives) {
      if (prim.extensions.draco) {
        // TODO: make our loader enforce Draco extension support
        // according to the spec.
        return Error("Draco extension is not supported in this build!");
      }
    }
  }
  return std::vector<BufferAccess>();
}

}  // namespace extensions
}  // namespace loader
}  // namespace imp
