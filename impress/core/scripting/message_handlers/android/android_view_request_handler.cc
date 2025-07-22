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
#include "core/common/platform_helpers.h"
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
          message.blend_priority)
      .Then([](ComponentHandle<AndroidViewRenderer> component) {
        return absl::OkStatus();
      });
}

AndroidViewGetAttachedViewRequestHandler::
    AndroidViewGetAttachedViewRequestHandler(BaseView& base_view)
    : base_view_(base_view) {}

Future<absl::Status> AndroidViewGetAttachedViewRequestHandler::HandleMessage(
    const GetAttachedViewRequest& message) {
  IMP_LOG(imp::FATAL) << "Wrong version of HandleMessage was called.";
  return Future<absl::Status>(
      absl::InternalError("Wrong version of HandleMessage was called."));
}

Future<absl::Status> AndroidViewGetAttachedViewRequestHandler::HandleMessage(
    const GetAttachedViewRequest& message, const scripting::PlatformArgs& args,
    scripting::PlatformArgs& out) {
  if (!message.target) {
    IMP_LOG(imp::FATAL) << "Invalid node";
    return Future<absl::Status>(absl::InvalidArgumentError("Invalid node"));
  }
  ComponentHandle<AndroidViewRenderer> android_view_renderer =
      message.target->GetComponent<AndroidViewRenderer>();
  if (!android_view_renderer) {
    // It is still required to populate the out list or the scripting
    // infrastructure will crash.
    out.push_back(nullptr);
    return Future<absl::Status>(
        absl::InvalidArgumentError("The given node has no child with an "
                                   "attached AndroidViewRenderer component."));
  }
  out.push_back(static_cast<void*>(android_view_renderer->GetAndroidView()));
  return Future<absl::Status>(absl::OkStatus());
}

AndroidViewUpdateColliderRequestHandler::
    AndroidViewUpdateColliderRequestHandler(BaseView& base_view)
    : base_view_(base_view) {}

Future<absl::Status> AndroidViewUpdateColliderRequestHandler::HandleMessage(
    const UpdateSurfaceTextureQuadColliderRequest& message) {
  IMP_LOG(imp::FATAL) << "Wrong version of HandleMessage was called.";
  return Future<absl::Status>(
      absl::InternalError("Wrong version of HandleMessage was called."));
}

Future<absl::Status> AndroidViewUpdateColliderRequestHandler::HandleMessage(
    const UpdateSurfaceTextureQuadColliderRequest& message,
    const scripting::PlatformArgs& args) {
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

}  // namespace android
}  // namespace imp
