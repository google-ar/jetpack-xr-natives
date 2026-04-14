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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "zetasql/base/init_google.h"
#include "absl/debugging/failure_signal_handler.h"
#include "absl/debugging/symbolize.h"
#include "absl/flags/flag.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "filament/filament/include/filament/Engine.h"
#include "third_party/grpc/include/grpcpp/client_context.h"
#include "third_party/grpc/include/grpcpp/create_channel.h"
#include "third_party/grpc/include/grpcpp/security/credentials.h"
#include "core/common/context.h"
#include "core/config.h"
#include "core/input/input_manager.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/android/split_engine_android_shared_memory_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_sender.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge.grpc.pb.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_client_impl.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_legacy.h"
#include "core/split_engine/desktop/multimachine/split_engine_desktop_bridge_sender.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge.grpc.pb.h"
#include "core/split_engine/desktop/split_engine_desktop_bridge_client.h"
#include "core/split_engine/split_engine_serializer_impl.h"
#include "core/view/framework/render/renderable_manager_wrapper.h"
#include "core/view/framework/view.h"
#include "core/view/utils/device.h"
#include "core/view/view_host.h"
#include "core/window/filament_host.h"
#include "core/window/sdl_venue.h"
#include "split_engine/schemas/split_engine_schema_version.h"
#include "mediapipe/framework/port/status_macros.h"

ABSL_FLAG(std::string, server, "localhost:10000",
          "address of server to connect to");
ABSL_FLAG(bool, multimachine, false,
          "Whether to use multimachine Split Engine.");

// Same as the default bridge buffer size on Android.
static constexpr size_t kBridgeBufferSizeBytes = 10000 * 1024;

namespace imp::split_engine {
namespace {

using window::FilamentHost;

absl::Status PresentHost(FilamentHost* host, InputManager* inputManager,
                         Device* device) {
#if IMP_RUNTIME(DEV)
  // Enable rendering for the editor.
  MP_RETURN_IF_ERROR(host->Setup());
#else
  // Nothing has to render, so we can use the NOOP backend.
  MP_RETURN_IF_ERROR(host->Setup(filament::Engine::Backend::NOOP));
#endif
  MP_RETURN_IF_ERROR(window::SdlVenue(host, inputManager, device));
  return absl::OkStatus();
}

absl::Status PresentHostAndCleanup(FilamentHost* host,
                                   InputManager* inputManager, Device* device) {
  if (auto status = PresentHost(host, inputManager, device); !status.ok()) {
    if (auto cleanup_status = host->Cleanup(); !cleanup_status.ok()) {
      return absl::InternalError(absl::StrFormat(
          "%.*s after earlier error %.*s",
          static_cast<int>(cleanup_status.message().size()),
          cleanup_status.message().data(),
          static_cast<int>(status.message().size()), status.message().data()));
    }
    return status;
  }
  MP_RETURN_IF_ERROR(host->Cleanup());
  return absl::OkStatus();
}

// Returns the API level to use for the serializer.
constexpr int32_t GetApiLevel() {
#ifdef IMP_SPLIT_ENGINE_ALLOW_EXPERIMENTAL_APIS
  return android_xr::kSplitEngineExperimentalApiLevel;
#else
  return android_xr::kSplitEngineProductionApiLevel;
#endif
}

absl::Status Main(int argc, char* argv[]) {
  // Connect to the server
  // TODO: (broken link) - update channel creation to use different credentials
  // based on the outcome of the bug.
  std::shared_ptr<grpc::Channel> channel = CreateChannel(
      absl::GetFlag(FLAGS_server), grpc::InsecureChannelCredentials());
  MP_ASSIGN_OR_RETURN(std::unique_ptr<SplitEngineDesktopBridgeClient> client,
                   SplitEngineDesktopBridgeClient::Create(
                       SplitEngineDesktopBridge::NewStub(channel)));

  auto bridge_sender =
      std::make_unique<SplitEngineSharedMemoryBridgeSender>(*client);

  std::unique_ptr<SplitEngineAndroidSharedMemoryBridge> bridge =
      std::make_unique<SplitEngineAndroidSharedMemoryBridge>(std::move(client));

  std::unique_ptr<View> view =
      imp::View::CreateClient(std::make_unique<Context>(argc, argv));

  view->SetRenderableManager(
      std::make_unique<imp::RenderableManagerWrapper>(*view));

  constexpr int32_t api_level = GetApiLevel();
  IMP_LOG(imp::INFO) << "Serializer using API level: "
            << (api_level == android_xr::kSplitEngineExperimentalApiLevel
                    ? "experimental"
                    : absl::StrCat(api_level));

  auto split_engine_serializer = std::make_unique<SplitEngineSerializerImpl>(
      *view, api_level, std::move(bridge), std::move(bridge_sender),
      kBridgeBufferSizeBytes);

  view->SetSplitEngineSerializer(std::move(split_engine_serializer));

  auto view_host = std::make_unique<ViewHost>(std::move(view));

  MP_RETURN_IF_ERROR(PresentHostAndCleanup(
      view_host.get(), &view_host->GetView()->GetInputManager(),
      &view_host->GetView()->GetDevice()));

  return absl::OkStatus();
}

absl::Status MultimachineMain(int argc, char* argv[]) {
  // Connect to the server
  // TODO: (broken link) - update channel creation to use different credentials
  // based on the outcome of the bug.
  std::shared_ptr<grpc::Channel> channel = CreateChannel(
      absl::GetFlag(FLAGS_server), grpc::InsecureChannelCredentials());
  std::unique_ptr<SplitEngineMMDesktopBridgeClient> client =
      std::make_unique<SplitEngineMMDesktopBridgeClientImpl>(
          SplitEngineMMDesktopBridge::NewStub(channel));

  auto bridge_sender =
      std::make_unique<SplitEngineMMDesktopBridgeSender>(*client);

  std::unique_ptr<SplitEngineAndroidBridge> bridge =
      std::make_unique<SplitEngineMMDesktopBridgeLegacy>(std::move(client));

  std::unique_ptr<View> view =
      imp::View::CreateClient(std::make_unique<Context>(argc, argv));

  view->SetRenderableManager(
      std::make_unique<imp::RenderableManagerWrapper>(*view));

  constexpr int32_t api_level = GetApiLevel();
  IMP_LOG(imp::INFO) << "Serializer using API level: "
            << (api_level == android_xr::kSplitEngineExperimentalApiLevel
                    ? "experimental"
                    : absl::StrCat(api_level));

  auto split_engine_serializer = std::make_unique<SplitEngineSerializerImpl>(
      *view, api_level, std::move(bridge), std::move(bridge_sender),
      kBridgeBufferSizeBytes);

  view->SetSplitEngineSerializer(std::move(split_engine_serializer));

  auto view_host = std::make_unique<ViewHost>(std::move(view));

  MP_RETURN_IF_ERROR(PresentHostAndCleanup(
      view_host.get(), &view_host->GetView()->GetInputManager(),
      &view_host->GetView()->GetDevice()));

  return absl::OkStatus();
}

}  // namespace
}  // namespace imp::split_engine

int main(int argc, char* argv[]) {
  absl::InitializeSymbolizer(argv[0]);

  absl::FailureSignalHandlerOptions options;
  absl::InstallFailureSignalHandler(options);

  InitGoogle("(--help for usage)", &argc, &argv, true);

  if (absl::GetFlag(FLAGS_multimachine)) {
    IMP_LOG(imp::ERROR) << "Running Multimachine variant.";
    if (const absl::Status status =
            imp::split_engine::MultimachineMain(argc, argv);
        !status.ok()) {
      IMP_LOG(imp::ERROR) << status;
      return -1;
    }
  } else {
    IMP_LOG(imp::ERROR) << "Running Singlemachine variant.";
    if (const absl::Status status = imp::split_engine::Main(argc, argv);
        !status.ok()) {
      IMP_LOG(imp::ERROR) << status;
      return -1;
    }
  }
  return 0;
}
