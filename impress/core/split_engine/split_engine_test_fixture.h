/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_FIXTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_FIXTURE_H_

#include <cassert>
#include <cstdint>
#include <memory>
#include <utility>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/base/thread_annotations.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/synchronization/notification.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "core/async/executor.h"
#include "core/common/enum_flags.h"
#include "core/common/invocable.h"
#include "core/common/owned_ptr.h"
#include "core/lighting/environment_light.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client_mock.h"
#include "core/split_engine/renderer_policy_handler_mock.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/split_engine/split_engine_renderer_impl.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/split_engine/split_engine_serializer_impl.h"
#include "core/split_engine/split_engine_serializer_transport_legacy_impl.h"
#include "core/split_engine/split_engine_test_bridge.h"
#include "core/split_engine/split_engine_test_bridge_serializer.h"
#include "core/view/base_view.h"
#include "core/view/framework/view.h"
#include "testing/base_scuba_view_fixture.h"
#include "testing/test_view.h"
#include "thread/thread.h"

namespace imp::split_engine {

using ::testing::_;
using ::testing::Return;
using ::testing::TestWithParam;

// An integration / unit test for Split Engine w/ a serializer and renderer.
//
// Example usage:
//
// class MyTest : public SplitEngineTestFixture<> {};
//
// TEST_P(MyTest, MyTest) {
//   // This blocks until the operations on the serializer view are complete.
//   RunOnSerializerView([](testing::GenericTestView<View>& view) {
//     // Do stuff on the serializer view.
//     Future<NodeHandle> model_future =
//         view.GetView()->GetAssetManager().LoadModel(kHumanSkeletonGlb);
//     view.GetFuture(model_future);
//   });
//   // Do stuff on the renderer view, i.e. take a screenshot of the scene.
//   // By this point, the serializer view should have sent all its messages to
//   // the renderer view.
//   EXPECT_THAT(RenderAndDiffGolden("MyTest.png"), testing::GoldenPassed());
//   // Note: must call ClearAppContent to avoid msan errors.
//   EXPECT_OK(split_engine_renderer_->ClearAppContent(kBridgeId));
// }
//
// INSTANTIATE_TEST_SUITE_P(
//     AllSchemaVersions, MyTest,
//     ::testing::Values(android_xr::kSplitEngineSchemaVersionCurrent));
//
// Note: the test must be parameterized with the schema version string or the
// RunOnSerializerView method will fail to compile.
template <typename RendererViewT = View>
class SplitEngineTestFixture
    : public testing::GenericScubaViewFixture<RendererViewT,
                                              TestWithParam<int32_t>> {
 public:
  // Note: These IDs are unique identifiers for the bridge on the client side
  // and service side, respectively. The client ID is used to identify message
  // groups and the bridge ID is used to identify the app context in the
  // renderer. Using the same values for now to avoid confusion, however they
  // could be different.
  static constexpr BridgeId kBridgeId = 123;
  static constexpr ClientId kClientId = 123;

  class SerializerView : public View {
   public:
    SerializerView() {
      // Disable default load to wait for the serializer to become available.
      GetLightManager().DisableDefaultLoad();
    }
  };

  using SerializerTestView = testing::GenericTestView<SerializerView>;

  // Class to hold the serializer-side imp::View and its associated objects.
  class SerializerViewHolder {
   public:
    SerializerViewHolder(
        SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer,
        int32_t api_level) {
      ON_CALL(bridge_client_, GetClientId).WillByDefault(Return(kClientId));
      ON_CALL(bridge_client_, GenerateMessageGroupId)
          .WillByDefault(::testing::InvokeWithoutArgs([]() {
            static int32_t message_group_id = 0;
            return ++message_group_id;
          }));

      auto bridge = std::make_unique<TestSplitEngineAndroidBridge>(
          bridge_client_, split_engine_test_bridge_serializer);

      auto sender = std::make_unique<TestSplitEngineBridgeSender>(*bridge);

      auto transport = imp::MakeOwned<SplitEngineSerializerTransportLegacyImpl>(
          std::move(bridge), std::move(sender));
      sender_ = sender.get();
      auto split_engine_serializer_impl =
          std::make_unique<split_engine::SplitEngineSerializerImpl>(
              *serializer_view_.GetView(), api_level, std::move(transport),
              // default shared memory size is ~10MB, same as in
              // ImpSplitEngineApi
              1024 * 10000);
      split_engine_serializer_ = split_engine_serializer_impl.get();

      serializer_view_.GetView()->SetSplitEngineSerializer(
          std::move(split_engine_serializer_impl));
    }

    SerializerTestView& GetView() { return serializer_view_; }

    SplitEngineSerializer* split_engine_serializer_;

    MockSplitEngineSharedMemoryBridgeClient bridge_client_;
    TestSplitEngineBridgeSender* sender_;

    SerializerTestView serializer_view_;
  };

  // To run Split Engine tests, we need to run the serializer-side view on a
  // separate thread. This more faithfully simulates the real-world scenario
  // where the serializer is running in a separate process from the renderer and
  // more importantly, allows the serializer to have its own filament engine.
  //
  // Tests should use RunOnSerializerView to run tasks on the serializer view,
  // which performs the given task on the serializer thread and blocks until the
  // task is complete.
  class SerializerViewThread : public Thread {
   public:
    SerializerViewThread(
        SplitEngineTestFixture& test,
        SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer,
        int32_t api_level)
        : test_(test),
          split_engine_test_bridge_serializer_(
              split_engine_test_bridge_serializer),
          api_level_(api_level) {}

    // Run the given task on the serializer view.
    // This is a blocking call that runs on a separate thread and all operations
    // on the serializer view shouldn't use the renderer view (i.e. GetFuture
    // must be called on the serializer view for assets loaded on the serializer
    // view, not the renderer view).
    void RunOnSerializerView(Invocable<void(SerializerTestView&)> task,
                             bool advance = true) {
      absl::Notification notification;
      {
        absl::MutexLock lock(mutex_);
        next_task_ = [task = std::move(task), &notification,
                      advance](SerializerTestView& view) {
          task(view);
          if (advance) {
            // Advance so that the serializer sends its messages to the
            // renderer.
            view.GetView()->Advance(absl::Milliseconds(16));
          }
          notification.Notify();
        };
      }

      absl::Time then = absl::Now();
      while (!notification.HasBeenNotified()) {
        test_.Simulate();
        if (absl::Now() - then > absl::Seconds(60)) {
          IMP_LOG(imp::FATAL) << "RunOnSerializerView timeout waiting for notification.";
        }
      }
    }

    // Stop the serializer view thread, blocking until it is safe to destroy.
    void Stop() {
      {
        absl::MutexLock lock(mutex_);
        running_ = false;
      }
      auto then = absl::Now();
      while (!shutdown_notification_.HasBeenNotified()) {
        test_.Simulate();
        if (absl::Now() - then > absl::Seconds(60)) {
          IMP_LOG(imp::FATAL) << "RunOnSerializerView timeout waiting for notification.";
        }
      }
    }

   protected:
    // The main loop for the serializer view thread.
    void Run() override {
      while (true) {
        // This mutex protects both `running_` and `next_task_` here.
        // RunOnSerializerView will wait for task to complete, so there is
        // always just one task and it's okay to lock mutex here for the
        // duration of the task.
        absl::MutexLock lock(mutex_);
        if (!running_) break;
        if (next_task_) {
          if (!serializer_view_) {
            serializer_view_ = std::make_unique<SerializerViewHolder>(
                split_engine_test_bridge_serializer_, api_level_);
          }
          next_task_(serializer_view_->GetView());
          // Formally speaking, this thread can yield execution here and if no
          // mutex is held, then race might happen:
          // 1. Main thread writes to `next_task_` new value and yields.
          // 2. This thread resumes and overwrites the value with empty.
          // 3. Task that was submitted from main thread is never executed.
          next_task_ = {};
        }
      }

      serializer_view_.reset();
      shutdown_notification_.Notify();
    }

   private:
    std::unique_ptr<SerializerViewHolder> serializer_view_;
    absl::Mutex mutex_;
    Invocable<void(SerializerTestView&)> next_task_ ABSL_GUARDED_BY(mutex_);

    absl::Notification shutdown_notification_;
    bool running_ ABSL_GUARDED_BY(mutex_) = true;

    SplitEngineTestFixture& test_;
    SplitEngineTestBridgeSerializer& split_engine_test_bridge_serializer_;
    int32_t api_level_;
  };

  SplitEngineTestFixture(
      int32_t serializer_api_level, int32_t renderer_api_level,
      absl::string_view golden_folder_path =
          "third_party/impress/core/split_engine/scuba_goldens")
      : testing::GenericScubaViewFixture<RendererViewT, TestWithParam<int32_t>>(
            golden_folder_path),
        serializer_api_level_(serializer_api_level),
        renderer_api_level_(renderer_api_level) {}

 protected:
  void SetUp() override {
    // Renderer side setup.
    // Note: due to the testing namespace, we need to use `this->` to access the
    // testing::GenericTestView<ViewT> methods like GetView().
    auto split_engine_renderer_impl =
        std::make_unique<split_engine::SplitEngineRendererImpl>(
            *this->GetView());
    split_engine_renderer_ = split_engine_renderer_impl.get();
    split_engine_renderer_->SetValidationApiLevel(renderer_api_level_);

    // Create a mock renderer policy handler and set it on the renderer.
    auto mock_renderer_policy_handler =
        std::make_unique<MockRendererPolicyHandler>();
    mock_renderer_policy_handler_ = mock_renderer_policy_handler.get();
    split_engine_renderer_->SetRendererPolicyHandler(
        kBridgeId, std::move(mock_renderer_policy_handler));
    // Add default behavior for the mock renderer policy handler.
    ON_CALL(*mock_renderer_policy_handler_, HandleUserId(_, _))
        .WillByDefault(Return(absl::OkStatus()));
    ON_CALL(*mock_renderer_policy_handler_, ClearUserId(_))
        .WillByDefault(Return(absl::OkStatus()));
    ON_CALL(*mock_renderer_policy_handler_, SetPreferredEnvironmentLight(_))
        .WillByDefault([view = this->GetView()](
                           BorrowedEnvironmentLightPtr environment_light) {
          // Default light is not loaded on renderer view automatically.
          if (environment_light) {
            view->GetLightManager().SetEnvironmentLight(&*environment_light);
            // In turn, we need to make sure that the default directional light
            // is created.
            view->GetLightManager().GetOrCreateDefaultDirectionalLight();
          }
          return absl::OkStatus();
        });
    ON_CALL(*mock_renderer_policy_handler_, GetMediatedRenderablePriority(_))
        .WillByDefault([](uint8_t priority) { return priority; });

    split_engine_renderer_->AddAppPermission(
        kBridgeId,
        imp::ToFlags(AppPermissionTypes::kHasUnrestrictedSystemAccess));
    this->GetView()
        ->GetRegistry()
        .template Register<split_engine::SplitEngineRenderer>(
            std::move(split_engine_renderer_impl));

    split_engine_test_bridge_serializer_ =
        std::make_unique<SplitEngineTestBridgeSerializer>(
            *this->GetView(), *Executor::ForegroundExecutor());
    serializer_view_thread_ = std::make_unique<SerializerViewThread>(
        *this, *split_engine_test_bridge_serializer_, serializer_api_level_);

    serializer_view_thread_->Start();

    RunOnSerializerView([](SerializerTestView& view) {
      // Load default lighting manually.
      view.GetView()->GetLightManager().EnsureLighting();

      // Should be either kLoadInProgress or kReady.
      EXPECT_NE(view.GetView()->GetLightManager().GetDefaultLightingStatus(),
                LightManager::EnvironmentLightingStatus::kUnloaded);

      while (view.GetView()->GetLightManager().GetDefaultLightingStatus() !=
             LightManager::EnvironmentLightingStatus::kReady) {
        view.DrainAllExecutors();
      }
    });
  }

  void TearDown() override {
    // Ensure all app content is cleared on shutdown.
    EXPECT_OK(split_engine_renderer_->ClearAppContext(kBridgeId));
  }

  ~SplitEngineTestFixture() override { serializer_view_thread_->Stop(); }

  // Runs the task on the serializer view thread and blocks until complete.
  //
  // Note: this method must be called from a TestT with a GetParam() method that
  // returns the schema version via TEST_P, meaning the test needs to both use
  // SplitEngineTestFixture and TEST_P with schema version strings.
  void RunOnSerializerView(Invocable<void(SerializerTestView&)> task) {
    // First, run the task on the serializer view mediated by the test bridge
    // serializer. If the schema version is current, the task will be run and
    // the messages will be captured. Otherwise, the task will be skipped.
    serializer_view_thread_->RunOnSerializerView(
        [this, task = std::move(task)](SerializerTestView& view) mutable {
          split_engine_test_bridge_serializer_->RunAndCaptureTask(
              *this,
              [this, task = std::move(task), &view]() mutable { task(view); });
        });

    // Then, take a snapshot of the captured messages. This will either create
    // a new snapshot or play back the snapshot.
    // Note: we don't advance the serializer view here because we want to ensure
    // the SplitEngineTestBridgeSerializer captures all messages (so none are
    // sent after the snapshot is taken).
    serializer_view_thread_->RunOnSerializerView(
        [this](SerializerTestView& view) {
          split_engine_test_bridge_serializer_->TakeSnapshot();
        },
        /*advance=*/false);

    // Ensure the renderer processes the messages from the serializer.
    this->Simulate();

    // Ensure all executors are drained in case any async work remains.
    // Note: this is needed for the non-current schema version case, as
    // otherwise the result is black.
    this->DrainAllExecutors();
  }

  SplitEngineRenderer* split_engine_renderer_;
  std::unique_ptr<SplitEngineTestBridgeSerializer>
      split_engine_test_bridge_serializer_;
  std::unique_ptr<SerializerViewThread> serializer_view_thread_;
  MockRendererPolicyHandler* mock_renderer_policy_handler_;
  int32_t serializer_api_level_;
  int32_t renderer_api_level_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEST_FIXTURE_H_
