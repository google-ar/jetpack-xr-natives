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

#include "core/view/framework/assets/gltf_extension.h"

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/ncsb/component_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"

namespace imp {

Future<absl::Status> GltfExtension::Setup() {
  ComponentHandle<GltfRenderer> renderer =
      GetNode()->GetComponent<GltfRenderer>();

  return Setup(renderer);
}

Future<absl::Status> GltfExtension::Setup(
    ComponentHandle<GltfRenderer> renderer) {
  return SetupInternal(renderer);
}

}  // namespace imp
