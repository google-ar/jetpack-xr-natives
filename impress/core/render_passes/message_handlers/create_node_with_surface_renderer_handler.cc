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

#include "core/render_passes/message_handlers/create_node_with_surface_renderer_handler.h"

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render_passes/surface_renderer.h"
#include "core/view/base_view.h"
#include "core/view/scripting/script_message_handler.h"

#if IMP_PLATFORM(ANDROID)
#include <android/native_window_jni.h>
#endif  // IMP_PLATFORM(ANDROID)

namespace imp {

CreateNodeWithSurfaceRendererHandler::CreateNodeWithSurfaceRendererHandler(
    BaseView* base_view)
    : view_(base_view) {}

Future<NodeHandle> CreateNodeWithSurfaceRendererHandler::HandleMessage(
    const CreateNodeWithSurfaceRendererRequest& message) {
  IMP_LOG(imp::FATAL) << "Wrong version of HandleMessage was called.";
  return Future<NodeHandle>(
      absl::InternalError("Wrong version of HandleMessage was called."));
}

Future<NodeHandle> CreateNodeWithSurfaceRendererHandler::HandleMessage(
    const CreateNodeWithSurfaceRendererRequest& message,
    const scripting::PlatformArgs& args) {
  if (args.size() != 1) {
    return Future<NodeHandle>(
        absl::InvalidArgumentError("CreateNodeWithSurfaceRendererHandler: "
                                   "Expected exactly one argument."));
  }

  if (args[0] == nullptr) {
    return Future<NodeHandle>(absl::InvalidArgumentError(
        "CreateNodeWithSurfaceRendererHandler: "
        "Received nullptr as native window pointer"));
  }

  void* native_window = nullptr;

  NodeHandle surface_renderer_node_handle = view_->CreateNode();

#if IMP_PLATFORM(ANDROID)
  native_window = ANativeWindow_fromSurface(
      surface_renderer_node_handle->GetView().GetContext().GetJniEnv(),
      reinterpret_cast<jobject>(args[0]));
#else
  IMP_LOG(imp::FATAL) << "CreateNodeWithSurfaceRendererHandler: Unsupported Platform.";
#endif

  absl::StatusOr<ComponentHandle<SurfaceRenderer>> status = absl::UnknownError(
      "CreateNodeWithSurfaceRendererHandler: Status is not set. Please check "
      "code paths to make sure the message handled properly.");

  status = surface_renderer_node_handle->AddComponent<SurfaceRenderer>(
      native_window, message.settings);

  if (!status.ok()) {
    return Future<NodeHandle>(absl::InternalError(
        absl::StrCat("CreateNodeWithSurfaceRendererHandler: Failed to "
                     "add SurfaceRenderer Component due to ",
                     status.status().message())));
  }

  return Future<NodeHandle>(surface_renderer_node_handle);
}

}  // namespace imp
