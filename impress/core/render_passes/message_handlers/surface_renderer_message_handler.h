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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_MESSAGE_HANDLERS_SURFACE_RENDERER_MESSAGE_HANDLERS_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_MESSAGE_HANDLERS_SURFACE_RENDERER_MESSAGE_HANDLERS_H_

#include "absl/status/status.h"
#include "core/render_passes/proto/surface_renderer_scripting.proto.imp.h"
#include "core/render_passes/surface_renderer.h"
#include "core/scripting/multi_message_handler.h"

namespace imp {

using surface_renderer::SetCameraRequest;
using surface_renderer::SetGroupRequest;
using surface_renderer::SetViewPortSizeRequest;

class SurfaceRendererMessageHandler : public scripting::MultiMessageHandler {
 public:
  explicit SurfaceRendererMessageHandler(BaseView* base_view)
      : view_(base_view) {
    AddHandler<SetCameraRequest, absl::Status>(this);
    AddHandler<SetGroupRequest, absl::Status>(this);
    AddHandler<SetViewPortSizeRequest, absl::Status>(this);
  }

  Future<absl::Status> HandleMessage(const SetCameraRequest& message);
  Future<absl::Status> HandleMessage(const SetGroupRequest& message);
  Future<absl::Status> HandleMessage(const SetViewPortSizeRequest& message);

 protected:
  BaseView* view_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_MESSAGE_HANDLERS_SURFACE_RENDERER_MESSAGE_HANDLERS_H_
