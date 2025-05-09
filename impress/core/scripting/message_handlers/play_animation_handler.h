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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_PLAY_ANIMATION_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_PLAY_ANIMATION_HANDLER_H_

#include "core/scripting/message_handler.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/base_view.h"

namespace imp::scripting {

// A MessageHandler for PlayAnimationRequests to start playing a gltf animation.
class PlayAnimationHandler
    : public MessageHandler<PlayAnimationRequest, absl::Status> {
 public:
  explicit PlayAnimationHandler(BaseView& view);

  Future<absl::Status> HandleMessage(
      const PlayAnimationRequest& message) override;

  constexpr static absl::string_view kInvalidNodeError = "Invalid node";
  constexpr static absl::string_view kInvalidAnimationIndexError =
      "Invalid animation index";
  constexpr static absl::string_view kInvalidAnimationNameError =
      "Invalid animation name";
  constexpr static absl::string_view kMissingAnimationError =
      "Missing animation name or index";
  constexpr static absl::string_view kNoModelError =
      "The node does not represent a model";

 private:
  BaseView& view_;
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_PLAY_ANIMATION_HANDLER_H_
