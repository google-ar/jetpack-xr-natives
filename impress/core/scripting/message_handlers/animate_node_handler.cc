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

#include "core/scripting/message_handlers/animate_node_handler.h"

#include "absl/types/optional.h"
#include "core/ncsb/node_handle.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/framework/animation/gltf_animator.h"

namespace imp::scripting {

AnimateNodeHandler::AnimateNodeHandler(BaseView& base_view)
    : view_(base_view) {}

Future<absl::Status> AnimateNodeHandler::HandleMessage(
    const AnimateNodeRequest& message) {
  return Future<absl::Status>(absl::UnimplementedError("Not Implemented"));
}

}  // namespace imp::scripting
