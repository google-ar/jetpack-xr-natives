// Copyright 2026 Google LLC
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

#include "core/scene_handles/material_handle.h"

#include <string>
#include <utility>

#include "absl/status/status.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/proto_asset.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/materials/material.h"
#include "core/render/material_registry.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/render/material_definition.proto.imp.h"

namespace imp {

Future<absl::Status> MaterialHandle::AsyncPreload(BaseView& view) {
  if (GetUrl().empty()) {
    return Future<absl::Status>(absl::OkStatus());
  }

  MaterialRegistry& material_registry =
      view.GetRegistry().GetOrCreate<MaterialRegistry>();
  BorrowedMaterialPtr material = material_registry.GetMaterial(GetUrl());
  if (material) {
    AssignMaterial(GetUrl(), material);
    return Future<absl::Status>(absl::OkStatus());
  }

  std::string url = std::string(GetUrl());
  return view.GetAssetManager().LoadProto<MaterialDefinition>(url).Then(
      [&view, url, this](AssetPtr<ProtoAsset<MaterialDefinition>> proto_asset) {
        if (!proto_asset) {
          return Future<absl::Status>(absl::InternalError(
              "Failed to load MaterialDefinition proto. Please ensure "
              "that the proto asset is included in the build. Loading "
              "directly from disk is only supported in DEV builds."));
        }
        const MaterialDefinition& material_definition = proto_asset->GetProto();
        return view.GetMaterialFactory()
            .LoadMaterial(material_definition)
            .Then([&view, url, this,
                   proto_asset = std::move(proto_asset)](MaterialPtr material) {
              MaterialRegistry& material_registry =
                  view.GetRegistry().GetOrCreate<MaterialRegistry>();
              if (!material_registry.HasMaterial(url)) {
                material_registry.RegisterMaterial(url, std::move(material),
                                                   proto_asset->GetProto());
              }
              AssignMaterial(url, material_registry.GetMaterial(url));
            });
      });
}

}  // namespace imp
