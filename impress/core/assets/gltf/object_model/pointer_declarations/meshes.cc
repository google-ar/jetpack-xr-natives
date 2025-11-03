/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "core/assets/gltf/object_model/pointer_declarations/meshes.h"

#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/assets/gltf/object_model/token_parser.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"

namespace imp::gltf {

using PointerValue = PropertyPointer::PointerValue;

namespace {
using PointerDeclaration = PropertyPointer::PointerDeclaration;

constexpr absl::string_view kMeshesLengthToken = "meshes.length";

}  // namespace

std::vector<TokenParser> MeshesLengthPointerDeclaration::GetTokenParsers()
    const {
  return {std::string(kMeshesLengthToken)};
}

absl::StatusOr<PointerValue> MeshesLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InternalError("No GltfRenderer found on the glTF model node.");
  }
  return gltf_renderer->GetMeshCount();
}

absl::Status MeshesLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PointerValue value) const {
  return absl::FailedPreconditionError(
      "Setting meshes length is not supported.");
}

}  // namespace imp::gltf
