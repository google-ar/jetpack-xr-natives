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

#include <memory>

#include "core/common/context.h"
#include "core/proto/any.proto.imp.h"
#include "core/scripting/basic_api.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/api.proto.imp.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/scripting/proto/events.proto.imp.h"
#include "core/scripting/scripting_system.h"
#include "core/scripting/test_resources.h"
#include "core/view/view_host.h"
#include "testing/executor_test_helper.h"
#include "testing/view_fixture.h"
#include "third_party/llvm/llvm-project/compiler-rt/include/fuzzer/FuzzedDataProvider.h"

namespace imp::scripting {
namespace {

using ::imp::proto::PackAny;
using ::imp::testing::ExecutorTestHelper;

class TestView : public View {
 public:
};

static float3 GetRandomFloat3(FuzzedDataProvider* data_provider) {
  return float3(data_provider->ConsumeFloatingPoint<float>(),
                data_provider->ConsumeFloatingPoint<float>(),
                data_provider->ConsumeFloatingPoint<float>());
}

static quatf GetRandomQuatf(FuzzedDataProvider* data_provider) {
  return quatf(data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>());
}

static mat4f GetRandomMat4f(FuzzedDataProvider* data_provider) {
  return mat4f(data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>(),
               data_provider->ConsumeFloatingPoint<float>());
}

enum class MessageTypes {
  DestroyNode,
  LoadModel,
  SetNodeEnabled,
  SetParent,
  SetTransform,
  SetTransformMatrix,
  kMaxValue = SetTransformMatrix
};

static void FillRandomDestroyNodeRequest(FuzzedDataProvider* data_provider,
                                         BaseView* view,
                                         MessageToNative* message_to_native) {
  auto request = DestroyNodeRequest();
  if (data_provider->ConsumeBool()) {
    request.target = view->CreateNode();
  }
  message_to_native->content = *PackAny(request);
}

static void FillRandomLoadModelRequest(FuzzedDataProvider* data_provider,
                                       BaseView* view,
                                       MessageToNative* message_to_native) {
  auto request = LoadModelRequest();
  if (data_provider->ConsumeBool()) {
    request.remote_uri = data_provider->ConsumeRandomLengthString(100);
  } else if (data_provider->ConsumeBool()) {
    request.remote_uri = imp::scripting::test_data::kAvocadoGlb.GetUrl();
  }
  message_to_native->content = *PackAny(request);
}

static void FillRandomSetNodeEnabledRequest(
    FuzzedDataProvider* data_provider, BaseView* view,
    MessageToNative* message_to_native) {
  auto request = SetNodeEnabledRequest();
  if (data_provider->ConsumeBool()) {
    request.target = view->CreateNode();
  }
  if (data_provider->ConsumeBool()) {
    request.enabled = data_provider->ConsumeBool();
  }
  message_to_native->content = *PackAny(request);
}

enum ParentChoice { New, Same, Self, None, kMaxValue = None };

static void FillRandomSetParentRequest(FuzzedDataProvider* data_provider,
                                       BaseView* view,
                                       MessageToNative* message_to_native) {
  auto request = SetParentRequest();
  request.target = view->CreateNode();

  if (data_provider->ConsumeBool()) {
    request.target->SetParent(view->CreateNode());
  }

  // Test all possibilities (itself, same parent, new parent, leave blank).
  ParentChoice parent_choice = data_provider->ConsumeEnum<ParentChoice>();
  switch (parent_choice) {
    case New:
      request.parent = view->CreateNode();
      break;
    case Same:
      request.parent = request.target->GetParent();
      break;
    case Self:
      request.parent = request.target;
      break;
    case None:
      break;
  }

  message_to_native->content = *PackAny(request);
}

static void FillRandomSetTransformRequest(FuzzedDataProvider* data_provider,
                                          BaseView* view,
                                          MessageToNative* message_to_native) {
  auto request = SetTransformRequest();
  if (data_provider->ConsumeBool()) {
    request.target = view->CreateNode();
  }
  if (data_provider->ConsumeBool()) {
    request.translation = GetRandomFloat3(data_provider);
  }
  if (data_provider->ConsumeBool()) {
    request.rotation = GetRandomQuatf(data_provider);
  }
  if (data_provider->ConsumeBool()) {
    request.scale = GetRandomFloat3(data_provider);
  }
  message_to_native->content = *PackAny(request);
}

static void FillRandomSetTransformMatrixRequest(
    FuzzedDataProvider* data_provider, BaseView* view,
    MessageToNative* message_to_native) {
  auto request = SetTransformMatrixRequest();
  if (data_provider->ConsumeBool()) {
    request.target = view->CreateNode();
  }
  if (data_provider->ConsumeBool()) {
    request.transform = GetRandomMat4f(data_provider);
  }
  message_to_native->content = *PackAny(request);
}

// TODO: use DEFINE_BINARY_PROTO_FUZZER instead of hard-coded.
static void FillWithRandomMessage(FuzzedDataProvider* data_provider,
                                  BaseView* view,
                                  MessageToNative* message_to_native) {
  auto message_type = data_provider->ConsumeEnum<MessageTypes>();
  switch (message_type) {
    case MessageTypes::DestroyNode:
      FillRandomDestroyNodeRequest(data_provider, view, message_to_native);
      break;
    case MessageTypes::LoadModel:
      FillRandomLoadModelRequest(data_provider, view, message_to_native);
      break;
    case MessageTypes::SetNodeEnabled:
      FillRandomSetNodeEnabledRequest(data_provider, view, message_to_native);
      break;
    case MessageTypes::SetParent:
      FillRandomSetParentRequest(data_provider, view, message_to_native);
      break;
    case MessageTypes::SetTransform:
      FillRandomSetTransformRequest(data_provider, view, message_to_native);
      break;
    case MessageTypes::SetTransformMatrix:
      FillRandomSetTransformMatrixRequest(data_provider, view,
                                          message_to_native);
      break;
    default:
      break;
  }
}

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
  auto executor_test_helper = std::make_unique<ExecutorTestHelper>();

  auto view_host = std::make_unique<imp::ViewHost>(
      imp::View::Create<TestView>("ViewFixture"));

  imp::resources::ResourceManager::AddFallback(
      imp::materials::kCompiledImpDefaultGltfMaterialsZip.GetUrl(),
      imp::materials_embedded::kCompiledImpDefaultGltfMaterialsZip);

  auto mock_url_loader_ptr = std::make_unique<imp::testing::MockUrlLoader>();
  auto mock_url_loader = mock_url_loader_ptr.get();
  view_host->GetView()->GetAssetManager().SetUrlLoader(
      std::move(mock_url_loader_ptr));
  imp::testing::SetupFakeResource(
      mock_url_loader, test_data::kAvocadoGlb.GetUrl(), test_data::kAvocadoGlb);

  EXPECT_THAT(view_host->Setup(filament::Engine::Backend::NOOP),
              imp::IsNoError());
  constexpr int kGenericViewFixtureWidth = 540;
  constexpr int kGenericViewFixtureHeight = 960;
  view_host->Resize({kGenericViewFixtureWidth, kGenericViewFixtureHeight},
                    {1, 1});

  WebViewParams params;
  auto scripting_system = std::make_unique<ScriptingSystem>(
      view_host->GetView()->GetContext(), nullptr, params);

  AddBasicApiMessageHandlers(*scripting_system, *view_host->GetView());

  FuzzedDataProvider data_provider(data, size);

  for (int i = 0; i < data_provider.ConsumeIntegralInRange<int32_t>(1, 100);
       i++) {
    // Generate a random message.
    MessageToNative message_to_native;
    message_to_native.message_id = 1;
    FillWithRandomMessage(&data_provider, view_host->GetView(),
                          &message_to_native);

    // Do the test!
    ExecutorTestHelper::AwaitFuture(
        scripting_system->HandleMessage(message_to_native));
  }

  // Manually shutdown executors to prevent future work from being scheduled
  // and flush the filament engine prior to cleaning up the ViewHost, so
  // we can clean up all resources before the filament engine is destroyed.
  executor_test_helper->GetBackgroundExecutor()->Shutdown();
  auto engine = BaseView::GetSharedEngine();
  assert(engine);
  do {
    FlushEngineAndWait(engine);
  } while (executor_test_helper->GetForegroundExecutor()->Pump(true));
  executor_test_helper->GetForegroundExecutor()->Shutdown();

  // Shutdown everything in order.
  scripting_system.reset();
  EXPECT_OK(view_host->Cleanup());
  view_host.reset();
  executor_test_helper.reset();

  return 0;
}

}  // namespace
}  // namespace imp::scripting
