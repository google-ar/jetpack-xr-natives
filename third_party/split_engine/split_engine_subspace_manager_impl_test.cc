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

#include "split_engine/split_engine_subspace_manager_impl.h"

#include <cstdint>
#include <memory>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "absl/status/status.h"
#include "absl/status/status_matchers.h"
#include "core/async/executor.h"
#include "testing/view_fixture.h"
#include "split_engine/input/split_engine_input_event.h"
#include "split_engine/subspace_events.h"

namespace android_xr {
namespace {

using ::absl_testing::StatusIs;
using ::imp::testing::ViewFixture;
using ::testing::Not;

class SplitEngineSubspaceManagerImplTest : public ViewFixture {
 protected:
  void SetUp() override {
    ViewFixture::SetUp();
    subspace_manager_ =
        std::make_unique<SplitEngineSubspaceManagerImpl>(*view_);
  }

  std::unique_ptr<SplitEngineSubspaceManagerImpl> subspace_manager_;
};

TEST_F(SplitEngineSubspaceManagerImplTest,
       DestroyAllSubspacesOnForegroundThreadDoesNotCrash) {
  // Register a subspace
  const uint32_t subspace_1 = subspace_manager_->GetNextSubspaceId();
  subspace_manager_->RegisterSubspace(subspace_1, 0);
  const uint32_t subspace_2 = subspace_manager_->GetNextSubspaceId();
  subspace_manager_->RegisterSubspace(subspace_2, 0);

  // Pump the executor to ensure registration completes
  DrainAllExecutors();

  // Verify registration by checking if ForwardInputEvent returns something
  // other than NotFound.
  SplitEngineInputEvent input_event;
  EXPECT_THAT(subspace_manager_->ForwardInputEvent(subspace_1, input_event),
              Not(StatusIs(absl::StatusCode::kNotFound)))
      << "Subspace should exist after registration";
  EXPECT_THAT(subspace_manager_->ForwardInputEvent(subspace_2, input_event),
              Not(StatusIs(absl::StatusCode::kNotFound)))
      << "Subspace should exist after registration";

  // Verify OnSubspaceDestroyedEvent is sent
  bool subspace_1_destroyed = false;
  bool subspace_2_destroyed = false;
  auto connection = view_->GetDispatcher().Connect(
      [&subspace_1_destroyed, &subspace_2_destroyed, subspace_1,
       subspace_2](const OnSubspaceDestroyedEvent& event) {
        if (event.subspace_id == subspace_1) {
          subspace_1_destroyed = true;
        } else if (event.subspace_id == subspace_2) {
          subspace_2_destroyed = true;
        } else {
          FAIL() << "Unexpected subspace destroyed " << event.subspace_id;
        }
      });

  // Make sure that DestroyAllSubspaces will cause DestroySubspace to be
  // executed synchronously - this will lead to removing element from the map.
  // If DestroyAllSubspaces is calling DestroySubspace _while_ iterating, map
  // implementation will crash.
  ASSERT_EQ(imp::Executor::CurrentExecutor(),
            imp::Executor::ForegroundExecutor());

  // Destroy all subspaces
  subspace_manager_->DestroyAllSubspaces();

  DrainAllExecutors();

  EXPECT_TRUE(subspace_1_destroyed);
  EXPECT_TRUE(subspace_2_destroyed);

  // Verify destruction
  EXPECT_THAT(subspace_manager_->ForwardInputEvent(subspace_1, input_event),
              StatusIs(absl::StatusCode::kNotFound))
      << "Subspace 1 should have been destroyed";
  EXPECT_THAT(subspace_manager_->ForwardInputEvent(subspace_2, input_event),
              StatusIs(absl::StatusCode::kNotFound))
      << "Subspace 2 should have been destroyed";
}

TEST_F(SplitEngineSubspaceManagerImplTest,
       DestroyAllSubspacesOnBackgroundThreadDoesNotCrash) {
  // Register a subspace
  const uint32_t subspace_1 = subspace_manager_->GetNextSubspaceId();
  subspace_manager_->RegisterSubspace(subspace_1, 0);
  const uint32_t subspace_2 = subspace_manager_->GetNextSubspaceId();
  subspace_manager_->RegisterSubspace(subspace_2, 0);

  // Pump the executor to ensure registration completes
  DrainAllExecutors();

  // Verify registration by checking if ForwardInputEvent returns something
  // other than NotFound.
  SplitEngineInputEvent input_event;
  EXPECT_THAT(subspace_manager_->ForwardInputEvent(subspace_1, input_event),
              Not(StatusIs(absl::StatusCode::kNotFound)))
      << "Subspace should exist after registration";
  EXPECT_THAT(subspace_manager_->ForwardInputEvent(subspace_2, input_event),
              Not(StatusIs(absl::StatusCode::kNotFound)))
      << "Subspace should exist after registration";

  // Verify OnSubspaceDestroyedEvent is sent
  bool subspace_1_destroyed = false;
  bool subspace_2_destroyed = false;
  auto connection = view_->GetDispatcher().Connect(
      [&subspace_1_destroyed, &subspace_2_destroyed, subspace_1,
       subspace_2](const OnSubspaceDestroyedEvent& event) {
        if (event.subspace_id == subspace_1) {
          subspace_1_destroyed = true;
        } else if (event.subspace_id == subspace_2) {
          subspace_2_destroyed = true;
        } else {
          FAIL() << "Unexpected subspace destroyed " << event.subspace_id;
        }
      });

  // Make sure that DestroyAllSubspaces will cause DestroySubspace to be
  // executed synchronously - this will lead to removing element from the map.
  // If DestroyAllSubspaces is calling DestroySubspace _while_ iterating, map
  // implementation will crash.
  ASSERT_EQ(imp::Executor::CurrentExecutor(),
            imp::Executor::ForegroundExecutor());

  // Destroy all subspaces
  imp::Executor::BackgroundExecutor()->Schedule(
      [&]() { subspace_manager_->DestroyAllSubspaces(); });

  DrainAllExecutors();

  EXPECT_TRUE(subspace_1_destroyed);
  EXPECT_TRUE(subspace_2_destroyed);

  // Verify destruction
  EXPECT_THAT(subspace_manager_->ForwardInputEvent(subspace_1, input_event),
              StatusIs(absl::StatusCode::kNotFound))
      << "Subspace 1 should have been destroyed";
  EXPECT_THAT(subspace_manager_->ForwardInputEvent(subspace_2, input_event),
              StatusIs(absl::StatusCode::kNotFound))
      << "Subspace 2 should have been destroyed";
}

}  // namespace
}  // namespace android_xr
