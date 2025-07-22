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

#include "core/scripting/message_handlers/load_and_apply_environment_light_handler.h"

#include <jni.h>

#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/lighting/environment_light_factory.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/ncsb/node.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/platforms/android/wrappers/input_stream.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp::scripting {
Future<absl::Status>
LoadAndApplyEnvironmentLightFromInputStreamHandler::HandleMessage(
    const LoadAndApplyEnvironmentLightFromInputStreamRequest& message) {
  IMP_LOG(imp::FATAL) << "Wrong version of HandleMessage was called.";
  return Future<absl::Status>(absl::InternalError(
      "Wrong version of HandleMessage was called for "
      "LoadAndApplyEnvironmentLightFromInputStreamHandler."));
}

Future<absl::Status>
LoadAndApplyEnvironmentLightFromInputStreamHandler::HandleMessage(
    const LoadAndApplyEnvironmentLightFromInputStreamRequest& message,
    const PlatformArgs& args) {
  if (args.size() != 1) {
    return Future<absl::Status>(absl::InvalidArgumentError(
        "Expected an java input stream as platform args."));
  }
  auto input_stream = std::make_unique<InputStream>(
      view_.GetContext().GetJniEnv(),
      WrapJni(view_.GetContext().GetJniEnv(),
              reinterpret_cast<jobject>(args[0])));

  Future<AssetPtr<ImageBasedLightingAsset>> load_ibl_future =
      view_.GetAssetManager().LoadAsset<ImageBasedLightingAsset>(
          std::move(input_stream), message.ibl_key);

  return load_ibl_future.Then([this](
                                  AssetPtr<ImageBasedLightingAsset> ibl_asset) {
    if (imp::split_engine::SplitEngineSerializer* serializer =
            view_.GetSplitEngineSerializer();
        serializer != nullptr) {
      serializer->SetPreferredEnvironmentIblAsset(
          *ibl_asset->GetReflectionTexture()->GetTexture(),
          LightManager::kDefaultEnvironmentLightIntensity, {1, 1, 1});
    }
    view_.GetLightManager().SetEnvironmentLight(
        view_.GetEnvironmentLightFactory().CreateEnvironmentLight(
            ibl_asset, imp::LightManager::kDefaultEnvironmentLightIntensity));
  });
}

}  // namespace imp::scripting
