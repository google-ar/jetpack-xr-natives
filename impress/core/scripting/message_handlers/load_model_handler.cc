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

#include "core/scripting/message_handlers/load_model_handler.h"

#include "absl/status/status.h"
#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/ncsb/node_handle.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/scene/scene_system.h"

#if IMP_PLATFORM(ANDROID)
#include <jni.h>

#include <memory>
#include <utility>

#include "core/common/log.h"
#include "core/assets/asset_ptr.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/platforms/android/wrappers/input_stream.h"
#include "core/view/scripting/script_message_handler.h"
#endif

namespace imp::scripting {

namespace {
static constexpr absl::string_view kExtensionTextproto = ".textproto";
}

Future<NodeHandle> LoadModelHandler::HandleMessage(
    const LoadModelRequest& message) {
  return view_.GetAssetManager()
      .LoadModel(message.remote_uri)
      .Then([message](const NodeHandle& node) {
        node->SetEnabled(message.enabled);
        return node;
      });
}

Future<NodeHandle> LoadModelHandler::HandleMessage(
    const LoadSceneRequest& message) {
  Future<NodeHandle> future;
  if (absl::EndsWith(message.scene, kExtensionTextproto)) {
    future = view_.GetAssetManager()
                 .LoadResource(message.scene)
                 .Then([this, url = message.scene](
                           resources::Resource resource) -> Future<NodeHandle> {
                   return view_.GetSceneSystem().LoadSceneFromTextproto(
                       resource, url);
                 });
  } else {
    future = view_.GetSceneSystem().LoadScene(message.scene);
  }
  return future.Then([message](const NodeHandle& node) {
    node->SetEnabled(message.enabled);
    return node;
  });
}

#if IMP_PLATFORM(ANDROID)
Future<NodeHandle> LoadModelFromInputStreamHandler::HandleMessage(
    const LoadModelFromInputStreamRequest& message) {
  IMP_LOG(imp::FATAL) << "Wrong version of HandleMessage was called.";
  return Future<NodeHandle>(
      absl::InternalError("Wrong version of HandleMessage was called for "
                          "LoadModelFromInputStreamHandler."));
}

Future<NodeHandle> LoadModelFromInputStreamHandler::HandleMessage(
    const LoadModelFromInputStreamRequest& message, const PlatformArgs& args) {
  if (args.size() != 1) {
    return Future<NodeHandle>(absl::InvalidArgumentError(
        "Expected an java input stream as platform args."));
  }
  auto input_stream = std::make_unique<InputStream>(
      view_.GetContext().GetJniEnv(),
      WrapJni(view_.GetContext().GetJniEnv(),
              reinterpret_cast<jobject>(args[0])));

  Future<NodeHandle> node_future =
      view_.GetAssetManager()
          .LoadGltfAsset(std::move(input_stream), message.model_key)
          .Then([this](const AssetPtr<GltfAsset>& gltf_asset) {
            NodeHandle node = view_.CreateNode();
            node->AddComponent<GltfRenderer>(gltf_asset);
            return node;
          });

  return node_future.Then([message](const NodeHandle& node) {
    node->SetEnabled(message.enabled);
    return node;
  });
}
#endif

}  // namespace imp::scripting
