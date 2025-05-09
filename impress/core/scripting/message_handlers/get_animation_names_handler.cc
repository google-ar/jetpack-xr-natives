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

#include "core/scripting/message_handlers/get_animation_names_handler.h"

#include "core/view/framework/assets/gltf_renderer.h"

namespace imp::scripting {

Future<GetAnimationNamesResponse> GetAnimationNamesHandler::HandleMessage(
    const GetAnimationNamesRequest& message) {
  auto node = message.target;
  if (!node) {
    return Future<GetAnimationNamesResponse>(
        absl::InvalidArgumentError(kInvalidNodeError));
  }
  auto gltf_renderer = node->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return Future<GetAnimationNamesResponse>(
        absl::CancelledError(kNoModelError));
  }
  auto names = gltf_renderer->GetGltfAsset()->GetAnimNames();
  auto response = GetAnimationNamesResponse();
  response.names.insert(response.names.end(), names.begin(), names.end());
  return Future<GetAnimationNamesResponse>(response);
}

}  // namespace imp::scripting
