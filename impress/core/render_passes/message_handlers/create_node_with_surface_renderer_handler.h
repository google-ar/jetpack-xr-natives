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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_MESSAGE_HANDLERS_CREATE_NODE_WITH_SURFACE_RENDERER_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_MESSAGE_HANDLERS_CREATE_NODE_WITH_SURFACE_RENDERER_HANDLER_H_

#include "core/async/future.h"
#include "core/render_passes/proto/surface_renderer_scripting.proto.imp.h"
#include "core/scripting/message_handler.h"
#include "core/view/base_view.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp {

using surface_renderer::CreateNodeWithSurfaceRendererRequest;

// Handles a CreateNodeWithSurfaceRendererRequest to return the surface renderer
// node.
// TODO: Make MultiMessageHandler work with platform args so that
// we can move this handler into surface_renderer_message_handler.h as well.
class CreateNodeWithSurfaceRendererHandler
    : public imp::scripting::MessageHandler<
          CreateNodeWithSurfaceRendererRequest, NodeHandle> {
 public:
  explicit CreateNodeWithSurfaceRendererHandler(BaseView* base_view);

  Future<NodeHandle> HandleMessage(
      const CreateNodeWithSurfaceRendererRequest& message) override;

  Future<NodeHandle> HandleMessage(
      const CreateNodeWithSurfaceRendererRequest& message,
      const scripting::PlatformArgs& args) override;

 protected:
  BaseView* view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_MESSAGE_HANDLERS_CREATE_NODE_WITH_SURFACE_RENDERER_HANDLER_H_
