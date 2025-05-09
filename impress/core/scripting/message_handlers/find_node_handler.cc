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

#include "core/scripting/message_handlers/find_node_handler.h"

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/node_handle_message.h"
#include "core/ncsb/path_manager.h"
#include "core/proto/any.proto.imp.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/gltf_scene.h"

namespace imp::scripting {
namespace {
constexpr absl::string_view kDoubleSlash = "//";
}  // namespace

FindNodeHandler::FindNodeHandler(BaseView& view) : view_(view) {}

Future<NodeHandle> FindNodeHandler::HandleMessage(
    const FindNodeRequest& message) {
  NodeHandle node = view_.GetPathManager().Find(message.location_path);
  if (node) {
    return Future<NodeHandle>(node);
  }

  // If node isn't found in the scene graph, attempt to find it in the skeletons
  // in all the models.
  view_.GetComponentManager().ForEach<GltfRenderer>(
      [&node, &message](GltfRenderer* gltf_renderer) {
        if (node) {
          // return early if the node has already been found.
          return;
        }

        auto node_name = message.location_path;
        if (absl::StartsWith(node_name, kDoubleSlash)) {
          node_name = node_name.substr(kDoubleSlash.size());
        }

        // Attempt to find a skeleton that has the queried path first, as
        // calling GetOrCreateNode without first verifying that the wanted bone
        // exists in the GltfScene will result in a default-created one
        // instead.
        AssetPtr<GltfAsset> gltf_asset = gltf_renderer->GetGltfAsset();
        if (!gltf_asset) {
          IMP_LOG(imp::WARNING) << "GltfRenderer component found with no GltfAsset!";
          return;
        }

        const imp::model::ModelData& data = gltf_asset->GetModelData();
        auto it = data.Skeleton().first_bone_from_hash.find(Hash(node_name));
        if (it != data.Skeleton().first_bone_from_hash.end()) {
          // Once we confirm that the bone exists, we'll get the
          // GltfScene to surface that node for us.
          auto gltf_scene = gltf_renderer->GetNode()->GetComponent<GltfScene>();
          if (!gltf_scene) {
            IMP_LOG(imp::ERROR) << "Unable to find skeleton in this model.";
            return;
          }
          node = gltf_scene->GetOrCreateNodeFromBone(it->second);
        }
      });
  if (node) {
    return Future<NodeHandle>(node);
  }
  return Future<NodeHandle>(absl::NotFoundError("Requested node not found."));
}

}  // namespace imp::scripting
