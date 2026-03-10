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

#include "core/scripting/message_handlers/android/android_view_request_handler.h"

#include <jni.h>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/scripting/message_handlers/android/android_view_renderer.h"
#include "core/scripting/message_handlers/android/android_view_renderer.proto.imp.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp {
namespace android {

AndroidViewRequestHandler::AndroidViewRequestHandler(BaseView& base_view)
    : base_view_(base_view) {}

Future<absl::Status> AndroidViewRequestHandler::HandleMessage(
    const CreateSurfaceTextureQuadRequest& message) {
  IMP_LOG(imp::FATAL) << "Wrong version of HandleMessage was called.";
  return Future<absl::Status>(
      absl::InternalError("Wrong version of HandleMessage was called."));
}

Future<absl::Status> AndroidViewRequestHandler::HandleMessage(
    const CreateSurfaceTextureQuadRequest& message,
    const scripting::PlatformArgs& args) {
  if (args.size() != 1) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Expected a View as platform args."));
  }
  if (!message.target) {
    return Future<absl::Status>(absl::InvalidArgumentError("Invalid node"));
  }
  return message.target
      ->AddComponent<AndroidViewRenderer>(
          reinterpret_cast<jobject>(args[0]), message.view_size,
          message.input_forwarding_mode, message.material,
          message.blend_priority, message.corner_radius)
      .Then([](ComponentHandle<AndroidViewRenderer> component) {
        return absl::OkStatus();
      });
}

AndroidViewGetAttachedViewRequestHandler::
    AndroidViewGetAttachedViewRequestHandler(BaseView& base_view)
    : base_view_(base_view) {}

Future<void*> AndroidViewGetAttachedViewRequestHandler::HandleMessage(
    const GetAttachedViewRequest& message) {
  if (!message.target) {
    IMP_LOG(imp::FATAL) << "Invalid node";
    return Future<void*>(absl::InvalidArgumentError("Invalid node"));
  }
  ComponentHandle<AndroidViewRenderer> android_view_renderer =
      message.target->GetComponent<AndroidViewRenderer>();
  if (!android_view_renderer) {
    return Future<void*>(
        absl::InvalidArgumentError("The given node has no child with an "
                                   "attached AndroidViewRenderer component."));
  }
  return Future<void*>(
      static_cast<void*>(android_view_renderer->GetAndroidView()));
}

AndroidViewUpdateColliderRequestHandler::
    AndroidViewUpdateColliderRequestHandler(BaseView& base_view)
    : base_view_(base_view) {}

Future<absl::Status> AndroidViewUpdateColliderRequestHandler::HandleMessage(
    const UpdateSurfaceTextureQuadColliderRequest& message) {
  if (!message.target) {
    return Future<absl::Status>(absl::InvalidArgumentError("Invalid node"));
  }
  ComponentHandle<AndroidViewRenderer> android_view_renderer =
      message.target->GetComponent<AndroidViewRenderer>();
  if (!android_view_renderer) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("The given node has no child with an "
                                   "attached AndroidViewRenderer component."));
  }
  android_view_renderer->UpdateCollider(message.collider);
  return Future<absl::Status>(absl::OkStatus());
}

AndroidViewUpdateScrollFactorRequestHandler::
    AndroidViewUpdateScrollFactorRequestHandler(BaseView& base_view)
    : base_view_(base_view) {}

Future<absl::Status> AndroidViewUpdateScrollFactorRequestHandler::HandleMessage(
    const UpdateAndroidViewScrollFactorRequest& message) {
  if (!message.target) {
    return Future<absl::Status>(absl::InvalidArgumentError("Invalid node"));
  }
  ComponentHandle<AndroidViewRenderer> android_view_renderer =
      message.target->GetComponent<AndroidViewRenderer>();
  if (!android_view_renderer) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("The given node has no child with an "
                                   "attached AndroidViewRenderer component."));
  }
  android_view_renderer->UpdateScrollFactor(message.horizontal_scroll_factor,
                                            message.vertical_scroll_factor);
  return Future<absl::Status>(absl::OkStatus());
}

}  // namespace android
}  // namespace imp
