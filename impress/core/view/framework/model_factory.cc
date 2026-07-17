/*
 * Copyright 2026 Google LLC
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

#include "core/view/framework/model_factory.h"

#include <utility>

#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/async/future.h"
#include "core/common/trace.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/asset.h"

namespace imp {

ModelFactory::ModelFactory(BaseView* view) : view_(view) {}

Future<NodeHandle> ModelFactory::LoadModel(
    const AssetDefinition& asset_definition,
    absl::optional<GltfAsset::LoadOptions> options) {
  return LoadModel(asset_definition.GetUrl(), std::move(options));
}

Future<NodeHandle> ModelFactory::LoadModel(
    absl::string_view asset_url,
    absl::optional<GltfAsset::LoadOptions> options) {
  IMP_TRACE();
  NodeHandle node = view_->CreateNode();
  return node->AddComponent<GltfRenderer>(asset_url, std::move(options))
      .Then([node, this](
                const absl::StatusOr<ComponentHandle<GltfRenderer>>& statusor)
                -> absl::StatusOr<NodeHandle> {
        IMP_TRACE_BLOCK("Then");
        if (!statusor.ok()) {
          view_->DestroyNode(node);
          return statusor.status();
        }
        return node;
      });
}

Future<NodeHandle> ModelFactory::LoadModel(
    absl::Cord contents, absl::string_view asset_url,
    absl::optional<GltfAsset::LoadOptions> options) {
  IMP_TRACE();
  NodeHandle node = view_->CreateNode();
  return node
      ->AddComponent<GltfRenderer>(std::move(contents), asset_url,
                                   std::move(options))
      .Then([node, this](
                const absl::StatusOr<ComponentHandle<GltfRenderer>>& statusor)
                -> absl::StatusOr<NodeHandle> {
        IMP_TRACE_BLOCK("Then");
        if (!statusor.ok()) {
          view_->DestroyNode(node);
          return statusor.status();
        }
        return node;
      });
}

}  // namespace imp
