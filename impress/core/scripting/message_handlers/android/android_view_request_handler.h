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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ANDROID_ANDROID_VIEW_REQUEST_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ANDROID_ANDROID_VIEW_REQUEST_HANDLER_H_

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/scripting/message_handler.h"
#include "core/scripting/message_handlers/android/android_view_renderer.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp {
namespace android {

// A MessageHandler that creates a node with an AndroidViewRenderer component.
class AndroidViewRequestHandler
    : public scripting::MessageHandler<CreateSurfaceTextureQuadRequest,
                                       absl::Status> {
 public:
  explicit AndroidViewRequestHandler(BaseView& base_view);

  Future<absl::Status> HandleMessage(
      const CreateSurfaceTextureQuadRequest& message) override;

  Future<absl::Status> HandleMessage(
      const CreateSurfaceTextureQuadRequest& message,
      const scripting::PlatformArgs& args) override;

 private:
  BaseView& base_view_;
};

// A message handler for getting the Android View associated with a node.
// The node should be the node returned from a CreateSurfaceTextureQuadRequest.
// TODO: Combined with the above in a MultiMessageHandler.
class AndroidViewGetAttachedViewRequestHandler
    : public scripting::MessageHandler<GetAttachedViewRequest, void*> {
 public:
  explicit AndroidViewGetAttachedViewRequestHandler(BaseView& base_view);

  Future<void*> HandleMessage(const GetAttachedViewRequest& message) override;

 private:
  BaseView& base_view_;
};

// A message handler for updating the collider of a node with an Android View.
// The node should be the node returned from a CreateSurfaceTextureQuadRequest.
class AndroidViewUpdateColliderRequestHandler
    : public scripting::MessageHandler<UpdateSurfaceTextureQuadColliderRequest,
                                       absl::Status> {
 public:
  explicit AndroidViewUpdateColliderRequestHandler(BaseView& base_view);

  Future<absl::Status> HandleMessage(
      const UpdateSurfaceTextureQuadColliderRequest& message) override;

 private:
  BaseView& base_view_;
};

}  // namespace android
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLERS_ANDROID_ANDROID_VIEW_REQUEST_HANDLER_H_
