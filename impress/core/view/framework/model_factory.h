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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_MODEL_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_MODEL_FACTORY_H_

#include <optional>

#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/async/future.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/asset.h"

namespace imp {

class BaseView;

// ModelFactory is used to load models and attach them to Nodes.
class ModelFactory {
 public:
  explicit ModelFactory(BaseView* view);

  // Loads a gLTF model into a Node asynchronously from an asset definition.
  // The returned Node will have children that map to the gLTF hierarchy.
  // The node will also have a GltfRenderer attached to it that can be used
  // to access information about the model.
  //
  // The model may be cached on subsequent calls, in which case the callback
  // will be invoked immediately.
  //
  // If the model is unable to be loaded, then the resulting NodeHandle is
  // invalid.
  Future<NodeHandle> LoadModel(
      const AssetDefinition& asset_definition,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);

  // Loads a gLTF model into a Node asynchronously from a url.
  Future<NodeHandle> LoadModel(
      absl::string_view asset_url,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);

  // Loads a gLTF model into a Node asynchronously from an absl::Cord.
  Future<NodeHandle> LoadModel(
      absl::Cord contents, absl::string_view asset_url,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);

 private:
  BaseView* view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_MODEL_FACTORY_H_
