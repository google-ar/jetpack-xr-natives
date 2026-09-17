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

#include "core/gsplat/gsplat_file_loader.h"

#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/editor/editor.h"
#include "core/editor/file_loader_helper.h"
#include "core/editor/file_type.h"
#include "core/gsplat/gsplat_asset.h"
#include "core/gsplat/gsplat_renderer.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"
#include "core/view/framework/assets/asset_manager.h"

namespace imp {
namespace {
static constexpr absl::string_view kExtensionGsplat = ".ply";
}  // namespace

const editor::FileType kFileTypeGsplat =
    editor::FileType::Create({kExtensionGsplat});

Future<absl::Status> GsplatFileLoader::LoadNode(
    absl::string_view path, Invocable<void(NodeHandle)> placement_func) {
  editor::Editor& editor = *GetView().GetRegistry().Get<editor::Editor>();
  editor::BeginLoadingNode(editor, GetView());
  auto node = GetView().CreateNode();
  return GetView()
      .GetAssetManager()
      .LoadAsset<GSplatAsset>(path)
      .Then([node](AssetPtr<GSplatAsset> gsplat_asset) {
        return node->AddComponent<GSplatRenderer>(gsplat_asset);
      })
      .Then(
          [&editor, this, path = std::string(path),
           placement_func = std::move(placement_func)](
              absl::StatusOr<ComponentHandle<GSplatRenderer>> gsplat) mutable {
            editor::EndLoadingNode(
                editor, GetView(), path, (*gsplat)->GetNode(),
                std::move(placement_func), [](NodeHandle node) {
                  auto scene_metadata = node->GetComponent<SceneMetadata>();
                  scene_metadata->SetComponentAuthored(
                      GSplatRenderer::IsfInfo::kTypeUrlHash,
                      /*is_authored=*/true);
                });
          });
}

}  // namespace imp
