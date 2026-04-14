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

#include "core/view/framework/assets/material_factory.h"

#include <cstring>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/trace.h"
#include "core/materials/custom_material.h"
#include "core/materials/material.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/utils/asset.h"

namespace imp {

namespace {
bool IsPlaceholderSplitEngineMaterial(const filament::Material* material) {
  if (material) {
    // TODO: (broken link) - Find a better way to check the placeholder material.
    return strcmp(material->getName(), "Split Engine Placeholder") == 0;
  }
  return false;
}
}  // namespace

Future<MaterialPtr> MaterialFactory::LoadMaterial(
    const AssetDefinition& asset_definition) {
  return LoadMaterial(asset_definition.GetUrl());
}

Future<MaterialPtr> MaterialFactory::LoadMaterial(absl::string_view asset_url) {
  IMP_TRACE();
  return view_->GetAssetManager().LoadMaterial(asset_url).Then(
      [this](imp::AssetPtr<imp::MaterialAsset> material_asset) mutable {
        IMP_TRACE_BLOCK("Then");
        return CreateMaterial(material_asset);
      });
}

Future<MaterialPtr> MaterialFactory::LoadMaterial(
    const MaterialDefinition& material_definition) {
  IMP_TRACE();
  return view_->GetAssetManager()
      .LoadMaterial(material_definition.asset)
      .Then([this, material_definition](
                AssetPtr<MaterialAsset> material_asset) mutable {
        IMP_TRACE_BLOCK("Then");
        MaterialPtr material = CreateMaterial(material_asset);

        // Set all the parameters on the material.
        Future<absl::Status> parameters_future = SetMaterialParameters(
            *view_, material.get(), material_definition.parameters);

        // Make sure that the material future isn't complete until the
        // parameters are all assigned.
        // Wrap the material in a future because capturing move only types can't
        // be moved within the body of the lambda.
        // TODO: Could improve the future API to make this use case
        // cleaner.
        Future<MaterialPtr> result(std::move(material));
        return parameters_future.Then([result]() { return result.Move(); });
      });
}

MaterialPtr MaterialFactory::CreateMaterial(
    const AssetPtr<MaterialAsset>& material_asset) {
  if (!material_asset) {
    IMP_LOG(imp::FATAL) << "Cannot create material from invalid material asset";
  }

  auto material = absl::WrapUnique(new CustomMaterial(
      material_asset->GetFilamentMaterial()->createInstance(), material_asset));

  if (auto serializer = view_->GetSplitEngineSerializer();
      serializer && !IsPlaceholderSplitEngineMaterial(
                        material_asset->GetFilamentMaterial())) {
    serializer->AddMaterialInstance(material_asset->GetFilamentMaterial(),
                                    material->GetFilamentMaterialInstance());
    return serializer->CreateCustomMaterial(std::move(material));
  }

  return material;
}

MaterialPtr MaterialFactory::CreateMaterial(
    const filament::Material* material) {
  if (!material) {
    IMP_LOG(imp::FATAL) << "Cannot create material from invalid filament material";
  }

  return CreateMaterial(*material);
}

MaterialPtr MaterialFactory::CreateMaterial(
    const filament::Material& material) {
  return WrapMaterial(material.createInstance());
}

MaterialPtr MaterialFactory::WrapMaterial(
    filament::MaterialInstance* material_instance) {
  if (!material_instance) {
    IMP_LOG(imp::FATAL) << "Cannot create material from invalid filament material";
  }

  auto material = absl::WrapUnique(new CustomMaterial(material_instance, {}));

  if (auto serializer = view_->GetSplitEngineSerializer();
      serializer &&
      !IsPlaceholderSplitEngineMaterial(material_instance->getMaterial())) {
    serializer->AddMaterialInstance(material_instance->getMaterial(),
                                    material_instance);
    return serializer->CreateCustomMaterial(std::move(material));
  }

  return material;
}

Future<absl::Status> MaterialFactory::SetMaterialParameters(
    BaseView& view, Material* material,
    const std::vector<MaterialDefinition::Parameter>& parameters) {
  Future<absl::Status> parameters_future(absl::OkStatus());

  // Loop through each parameter to assign it.
  for (const MaterialDefinition::Parameter& parameter : parameters) {
    if (parameter.name.empty() || !material->HasParameter(parameter.name)) {
      IMP_LOG(imp::WARNING) << "Unable to set parameter \"" << parameter.name
                   << "\" on material " << material->GetName();
      continue;
    }

    if (parameter.registered_texture()) {
      // Special handling for textures from the TextureRegistry.
      material->SetParameter(parameter.name,
                             view.GetTextureRegistry().GetTexture(
                                 *parameter.registered_texture()));
    } else if (parameter.image_asset_texture()) {
      // Special handling for textures loaded asynchronously from image
      // assets.
      parameters_future = parameters_future.Combine(
          view.GetAssetManager()
              .LoadImage(*parameter.image_asset_texture())
              .Then([&view, material,
                     parameter](AssetPtr<ImageAsset> image_asset) mutable {
                OwnedTexturePtr texture =
                    view.GetTextureFactory().CreateTexture(image_asset);
                material->SetParameter(parameter.name, std::move(texture));
              }));
    } else {
      // Default handling for parameters.
      absl::visit(
          [material, &parameter](const auto& parameter_value) {
            using ParamT = std::decay_t<decltype(parameter_value)>;
            if constexpr (!std::is_same_v<ParamT, absl::monostate> &&
                          !std::is_same_v<ParamT, std::string>) {
              material->SetParameter(parameter.name, parameter_value);
            }
          },
          parameter.val);
    }
  }
  return parameters_future;
}

}  // namespace imp
