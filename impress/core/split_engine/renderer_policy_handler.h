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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_renderer_policy_handler_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_renderer_policy_handler_H_

#include <stdint.h>
#include <sys/types.h>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/lighting/environment_light.h"
#include "core/ncsb/node_handle.h"

namespace imp::split_engine {

// Interface that allows systems to plug into Split Engine to handle policy
// updates. The implementation details of RendererPolicyHandler is a system side
// concern.
class RendererPolicyHandler {
 public:
  virtual ~RendererPolicyHandler() = default;

  RendererPolicyHandler() {}

  // The call to HandleUserId will notify the system about a node's association
  // to the specified user_id.
  //
  /// NOTE: The user_id is an unsigned integer to match the incoming value type
  /// of the NodeUpdate's user_id property. It is expected to be unique within
  /// the context of a single application.
  virtual absl::Status HandleUserId(uint32_t user_id, NodeHandle node) = 0;

  // The call to ClearUserId will notify the system that the node is no longer
  // associated with the specified user_id
  virtual absl::Status ClearUserId(uint32_t user_id) = 0;

  // The call to SetPreferredEnvironmentLight will notify the system about
  // a Split Engine application's preferred EnvironmentLight. If
  // environment_light is null (i.e. empty), the Split Engine application has no
  // preferred EnvironmentLight and a fallback should be used.
  virtual absl::Status SetPreferredEnvironmentLight(
      BorrowedEnvironmentLightPtr environment_light) = 0;

  // Checks whether the given priority is allowed by the policy for the app,
  // returning an error if it is higher than the policy's maximum priority.
  // The returned value is the priority that should be used by the app and may
  // be higher than the requested priority for some special trusted apps.
  virtual absl::StatusOr<uint8_t> GetMediatedRenderablePriority(
      uint8_t priority) = 0;

  // Used by Split Engine to determine whether to allow raw binary precompiled
  // materials to be added to the scene. If you're unsure whether or not your
  // app needs these materials, then this method should probably just return
  // `false`. These materials are not allowed in production.
  virtual absl::Status AreBinaryPrecompiledMaterialsAllowed() {
    return absl::OkStatus();
  };
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_RENDERER_POLICY_HANDLER_H_
