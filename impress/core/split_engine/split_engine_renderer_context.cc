// Copyright 2025 Google LLC
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

#include "core/split_engine/split_engine_renderer_context.h"

#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/math/mat.h"
#include "core/ncsb/node.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/user_id_holder.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"

namespace imp::split_engine {

AppPermissionController::AppPermissionController(BaseView& view) : view_(view) {
  default_parent_node_ = view_.CreateNode();
  // Disable the default parent node by default.
  default_parent_node_->SetEnabled(false);
}

void AppPermissionController::AddPermissions(AppPermission permission) {
  app_permissions_ |= permission;
  Update();
}

void AppPermissionController::RemovePermissions(AppPermission permission) {
  app_permissions_ &= ~permission;
  Update();
}

bool AppPermissionController::HasPermission(
    AppPermissionTypes permission) const {
  return app_permissions_.Test(permission);
}

void AppPermissionController::Update() {
  default_parent_node_->SetEnabled(
      HasPermission(AppPermissionTypes::kHasUnrestrictedSystemAccess));
}

void AppPermissionController::ApplyControls(imp::NodeHandle node) {
  // If the node does not have a valid parent, i.e. the parent has been
  // destroyed or replaced by nullptr, set it to the default parent node that
  // is controlled by the permission controller.
  if (!node->GetParent()) {
    node->SetParent(default_parent_node_);
  }
  if (!app_permissions_.Test(
          AppPermissionTypes::kAllowCustomTransformsOnNodesWithUserIds) &&
      node->GetComponent<UserIdHolder>()) {
    // If the node has a UserIdHolder component, remove any updated transform
    // information, and does not have the permission to update its userId
    // transform, remove any incoming transforms.
    node->SetLocalTrs(imp::mat4f());
  }
}

imp::NodeHandle AppPermissionController::GetDefaultParentNode() {
  return default_parent_node_;
}

AppContext::AppContext(BaseView& view,
                       EnvironmentLightContext& environment_light_cxt,
                       BridgeId bridge_id)
    : view(view),
      environment_light_context(environment_light_cxt),
      app_permission_controller(view),
      bridge_id(bridge_id) {}

AppContext::~AppContext() {
  for (const auto& [entity_id, node] : entity_map) {
    view.DestroyNode(node);
  }

  // Material instances are type MaterialPtr, which have auto-delete
  // semantics.
  material_instances.clear();

  // Materials are type OwnedFilamentMaterialPtr, which have auto-delete
  // semantics.
  materials.clear();

  // This should auto-release the textures, as they are OwnedTexturePtr. Also
  // the destruction order matters: Textures must be released after materials,
  // as materials may be borrowing them.
  textures.clear();

  for (const auto& [texture_id, texture_metadata] : textures_external) {
    texture_metadata.release_fn();
  }
  textures_external.clear();

  // Mark all IBL assets as pending removal. They will be released in the
  // Update() function only when all borrows are released.
  for (auto it = environment_light_context.lights.begin(),
            end = environment_light_context.lights.end();
       it != end;) {
    // Note: this is the advised pattern for erasing an item from a map while
    // iterating through it based on the documentation of flat_hash_map.
    auto it_copy = it++;
    environment_light_context.pending_removes.push_back(
        std::move(environment_light_context.lights.extract(it_copy).mapped()));
  }
  // Mark the environment light context as destroyed so that it can be removed
  // from the environment_light_contexts_ set in the Update() function.
  environment_light_context.app_context_destroyed = true;
  // Notify the RendererPolicyHandler that the preferred environment light is
  // being cleared.
  if (renderer_policy_handler) {
    if (absl::Status status =
            renderer_policy_handler->SetPreferredEnvironmentLight({});
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to set preferred environment light to empty: "
                 << status;
    }
  }

  // TODO: (broken link) - Create a more targeted API for clearing assets.
  view.GetAssetManager().ClearUnused();
}

}  // namespace imp::split_engine
