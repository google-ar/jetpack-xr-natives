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

#include "core/scripting/message_handlers/play_animation_handler.h"

#include "absl/status/status.h"
#include "absl/types/variant.h"
#include "core/async/future.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/animation/gltf_animator.h"
#include "core/view/framework/assets/gltf_renderer.h"

namespace imp::scripting {

PlayAnimationHandler::PlayAnimationHandler(BaseView& view) : view_(view) {}

Future<absl::Status> PlayAnimationHandler::HandleMessage(
    const PlayAnimationRequest& message) {
  auto node = message.target;
  if (!node) {
    return Future<absl::Status>(absl::InvalidArgumentError(kInvalidNodeError));
  }
  auto gltf_renderer = node->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return Future<absl::Status>(absl::CancelledError(kNoModelError));
  }
  auto asset = gltf_renderer->GetGltfAsset();
  auto command = message.play_command;

  auto* index = absl::get_if<command.kAnimation_Index>(&command.animation);
  auto* name = absl::get_if<command.kAnimation_Name>(&command.animation);
  if (index != nullptr) {
    const auto& animations = asset->GetAnimNames();
    if (*index >= animations.size() || *index < 0) {
      return Future<absl::Status>(
          absl::InvalidArgumentError(kInvalidAnimationIndexError));
    }
  } else if (name != nullptr) {
    auto id = asset->GetAnimId(*name);
    if (!id) {
      return Future<absl::Status>(
          absl::InvalidArgumentError(kInvalidAnimationNameError));
    }
  } else {
    return Future<absl::Status>(
        absl::InvalidArgumentError(kMissingAnimationError));
  }

  auto animator = node->GetOrAddComponent<GltfAnimator>();

  animator->Play(message.play_command);
  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace imp::scripting
