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

#include "core/split_engine/materials/split_engine_builtin_material.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/common/owned_ptr.h"
#include "core/common/registry.h"
#include "core/common/robin_set.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/render/texture.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material_factory.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<OwnedMaterialPtr> SplitEngineBuiltinMaterial::RequestBuiltInMaterial(
    BaseView& view, std::unique_ptr<flatbuffers::FlatBufferBuilder> fbb,
    android_xr::schemas::BuiltInMaterialSpec material_type,
    flatbuffers::Offset<void> spec) {
  if (view.AreSplitEngineMaterialsInLocalMode()) {
    // Create a built-in material request with ID zero since it will not be
    // used.
    flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest>
        built_in_material_request =
            android_xr::schemas::CreateBuiltInMaterialRequest(
                *fbb, /*material_instance_id=*/0, material_type, spec);
    std::vector<uint8_t> data = SerializeTable(*fbb, built_in_material_request);
    const android_xr::schemas::BuiltInMaterialRequest* schema =
        flatbuffers::GetRoot<android_xr::schemas::BuiltInMaterialRequest>(
            data.data());
    // Local mode does not use the bridge, so we can use a fake bridge ID.
    const BridgeId fake_bridge_id = 1;
    return SplitEngineBuiltinMaterialFactory::HandleCreateRequest(
               view, fake_bridge_id, *schema)
        .Then([](BuiltInMaterialPtr material) -> Future<OwnedMaterialPtr> {
          return Future<OwnedMaterialPtr>(
              OwnedMaterialPtr{std::move(material)});
        });
  }

  return CreatePlaceholderMaterial(view).Then(
      [&view, fbb = std::move(fbb), material_type,
       spec](OwnedMaterialPtr placeholder_material) {
        flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest>
            built_in_material_request =
                android_xr::schemas::CreateBuiltInMaterialRequest(
                    *fbb,
                    SplitEngineSerializer::GetId(
                        placeholder_material->GetFilamentMaterialInstance()),
                    material_type, spec);
        return SendRequest<android_xr::schemas::BuiltInMaterialRequest,
                           absl::Status>(
                   view.GetSplitEngineSerializer()->GetBridge(), *fbb,
                   built_in_material_request)
            .Then([placeholder_material =
                       std::move(placeholder_material)]() mutable {
              return OwnedMaterialPtr(std::move(placeholder_material));
            });
      });
}

SplitEngineBuiltinMaterial::SplitEngineBuiltinMaterial(
    BaseView& view,
    android_xr::schemas::BuiltInMaterialParameters parameters_type,
    OwnedMaterialPtr material)
    : SplitEngineMaterial(view, std::move(material)),
      parameters_type_(parameters_type) {}

SplitEngineBuiltinMaterial::~SplitEngineBuiltinMaterial() {
  // Subclasses should override and call Cleanup().
  
  // In case of programmer error, call Cleanup() anyway.
  if (!cleanup_called_) {
    Cleanup();
  }
}

void SplitEngineBuiltinMaterial::Cleanup() {
  SplitEngineBuiltinMaterialUpdater& updater =
      view_.GetRegistry().GetOrCreate<SplitEngineBuiltinMaterialUpdater>(view_);
  updater.RemoveMaterial(this);
  if (material_) {
    if (SplitEngineSerializer* serializer = view_.GetSplitEngineSerializer()) {
      serializer->RemoveMaterialInstance(GetFilamentMaterialInstance());
    }
    material_.Reset();
  }
  cleanup_called_ = true;
}

BorrowedMaterialPtr SplitEngineBuiltinMaterial::GetMaterial(
    SmallSourceLocation loc) const {
  return material_.Borrow(loc);
}

void SplitEngineBuiltinMaterial::MarkParametersDirty(bool dirty) const {
  if (view_.AreSplitEngineMaterialsInLocalMode()) {
    if (dirty) {
      // If running in "local mode", update the parameters immediately.
      UpdateParameters();
      return;
    }
  } else {
    SplitEngineBuiltinMaterialUpdater& updater =
        view_.GetRegistry().GetOrCreate<SplitEngineBuiltinMaterialUpdater>(
            view_);
    updater.MarkParametersDirty(this, dirty);
  }
}

bool SplitEngineBuiltinMaterial::AreParametersDirty() const {
  SplitEngineBuiltinMaterialUpdater& updater =
      view_.GetRegistry().GetOrCreate<SplitEngineBuiltinMaterialUpdater>(view_);
  return updater.AreParametersDirty(this);
}

void SplitEngineBuiltinMaterial::UpdateParameters() const {
  const bool kUsingLocalMode = view_.AreSplitEngineMaterialsInLocalMode();
  BuiltInTextureParameterCreator texture_parameter_creator(kUsingLocalMode);
  auto serialize_func = [this, &texture_parameter_creator](
                            flatbuffers::FlatBufferBuilder& fbb) mutable {
    return SerializeParameters(fbb, texture_parameter_creator);
  };
  if (kUsingLocalMode) {
    // In local mode, the material is the real, locally-created built-in
    // material. The material will be serialized as a raw material, which
    // requires that the Filament version on device matches the app version.
    // TextureBorrower holds textures in a vector and its index is used for
    // retrieving the texture.
    TextureBorrower texture_borrower =
        [&texture_parameter_creator](
            uint64_t texture_id,
            SmallSourceLocation loc) -> BorrowedTexturePtr {
      return texture_parameter_creator.Borrow(texture_id, loc);
    };

    flatbuffers::FlatBufferBuilder fbb;
    flatbuffers::Offset<android_xr::schemas::BuiltInMaterialInstanceParameters>
        built_in_material_instance_parameters =
            android_xr::schemas::CreateBuiltInMaterialInstanceParameters(
                fbb,
                SplitEngineSerializer::GetId(GetFilamentMaterialInstance()),
                parameters_type_, serialize_func(fbb));
    std::vector<uint8_t> data =
        SerializeTable(fbb, built_in_material_instance_parameters);
    const android_xr::schemas::BuiltInMaterialInstanceParameters* schema =
        flatbuffers::GetRoot<
            android_xr::schemas::BuiltInMaterialInstanceParameters>(
            data.data());
    flatbuffers::Verifier verifier(data.data(), data.size());

    // In local mode, the material_ is supposed to be an OwnedMaterialPtr
    // holding a BuiltInMaterial, but there's an edge case where generic
    // material uses remote material even on local mode. Thus, we can't cast
    // directly to BuiltInMaterial.
    // TODO: Remove else block once (broken link) is fixed.
    if (IsAlwaysRemote()) {
      view_.GetSplitEngineSerializer()->SetBuiltInMaterialParameters(
          GetMaterial()->GetFilamentMaterialInstance(),
          static_cast<BuiltInMaterialParameters>(parameters_type_),
          std::move(serialize_func));
    } else {
      // In local mode, the material_ is an OwnedMaterialPtr holding a
      // BuiltInMaterial, so this cast is safe.
      BorrowedPtr<BuiltInMaterial> local_material =
          BorrowedPtr<BuiltInMaterial>(
              material_.Borrow(SmallSourceLocation::Current()));
      if (absl::Status status = local_material->SetParameters(verifier, *schema,
                                                              texture_borrower);
          !status.ok()) {
        IMP_LOG(imp::FATAL) << "Failed to set parameters on built-in material: "
                   << status;
      }
    }
  } else {
    // In Split Engine built-in material mode, the OwnedMaterialPtr is a
    // placeholder material. Serialization of the material parameters happens
    // through the built-in schema to ensure safety & backwards compatibility.
    view_.GetSplitEngineSerializer()->SetBuiltInMaterialParameters(
        GetFilamentMaterialInstance(),
        static_cast<BuiltInMaterialParameters>(parameters_type_),
        std::move(serialize_func));
  }
}

SplitEngineBuiltinMaterialUpdater::SplitEngineBuiltinMaterialUpdater(
    BaseView& view)
    : Updater(view) {}

void SplitEngineBuiltinMaterialUpdater::Update(const FrameTime& frame_time) {
  for (const SplitEngineBuiltinMaterial* material : dirty_) {
    material->UpdateParameters();
  }
  dirty_.clear();
}

void SplitEngineBuiltinMaterialUpdater::RemoveMaterial(
    const SplitEngineBuiltinMaterial* material) {
  dirty_.erase(material);
}

void SplitEngineBuiltinMaterialUpdater::MarkParametersDirty(
    const SplitEngineBuiltinMaterial* material, bool dirty) {
  if (dirty) {
    dirty_.insert(material);
  } else {
    dirty_.erase(material);
  }
}

bool SplitEngineBuiltinMaterialUpdater::AreParametersDirty(
    const SplitEngineBuiltinMaterial* material) const {
  return dirty_.contains(material);
}

}  // namespace imp::split_engine
