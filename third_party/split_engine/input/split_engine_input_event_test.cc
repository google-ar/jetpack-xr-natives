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

#include "split_engine/input/split_engine_input_event.h"

#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "split_engine/input/split_engine_input_event_hit_info.h"

namespace android_xr {

class SplitEngineInputEventTest : public ::testing::Test {
 public:
  // Initializes the given event to arbitrary values for testing.
  //
  // The `hit_position` in `hit_info` and `secondary_hit_info` will be set,
  // but none of the other SplitEngineInputEventHitInfo fields.
  static void InitToTestValues(SplitEngineInputEvent* event) {
    event->dispatch_flag = SplitEngineInputEvent::DispatchFlag::TWO_D;
    event->device_type = SplitEngineInputEvent::DeviceType::CONTROLLER;
    event->pointer_type = SplitEngineInputEvent::PointerType::EYE;

    event->timestamp_ms = 100000000000L;

    event->origin.x = 1.0f;
    event->origin.y = 2.0f;
    event->origin.z = 3.0f;

    event->direction.x = 4.0f;
    event->direction.y = 5.0f;
    event->direction.z = 6.0f;

    event->button_state = 1;

    event->hit_node = std::make_unique<SplitEngineInputEventHitInfo>();
    event->hit_node->hit_position.x = 3.0f;
    event->hit_node->hit_position.y = 2.0f;
    event->hit_node->hit_position.z = 1.0f;

    event->secondary_hit_node =
        std::make_unique<SplitEngineInputEventHitInfo>();
    event->secondary_hit_node->hit_position.x = 6.0f;
    event->secondary_hit_node->hit_position.y = 5.0f;
    event->secondary_hit_node->hit_position.z = 4.0f;
  }
};

TEST(SplitEngineInputEventTest, CopyAssignmentAssignsAllFields) {
  SplitEngineInputEvent event_a;
  SplitEngineInputEvent event_b;

  // Initialize 'b', copy into default initialized 'a'.
  SplitEngineInputEventTest::InitToTestValues(&event_b);
  event_a = event_b;

  // Verify that 'a' is set after copy assignment.
  EXPECT_EQ(event_a.dispatch_flag, event_b.dispatch_flag);
  EXPECT_EQ(event_a.device_type, event_b.device_type);
  EXPECT_EQ(event_a.pointer_type, event_b.pointer_type);
  EXPECT_EQ(event_a.timestamp_ms, event_b.timestamp_ms);
  EXPECT_EQ(event_a.origin, event_b.origin);
  EXPECT_EQ(event_a.direction, event_b.direction);
  EXPECT_EQ(event_a.button_state, event_b.button_state);

  // InitToTestValues only sets `hit_position`, so check that field.
  EXPECT_EQ(event_a.hit_node->hit_position, event_b.hit_node->hit_position);
  EXPECT_EQ(event_a.secondary_hit_node->hit_position,
            event_b.secondary_hit_node->hit_position);
}

TEST(SplitEngineInputEventTest, CopyAssignmentClearsAllFields) {
  SplitEngineInputEvent event_a;
  SplitEngineInputEvent event_b;

  // Initialize 'a', copy default initialized 'b' into 'a'.
  SplitEngineInputEventTest::InitToTestValues(&event_a);
  event_a = event_b;

  // Verify that 'a' is cleared after copy assignment.
  EXPECT_EQ(event_a.dispatch_flag, SplitEngineInputEvent::DispatchFlag::NONE);
  EXPECT_EQ(event_a.device_type, SplitEngineInputEvent::DeviceType::UNKNOWN);
  EXPECT_EQ(event_a.pointer_type, SplitEngineInputEvent::PointerType::DEFAULT);
  EXPECT_EQ(event_a.timestamp_ms, 0L);

  // EXPECT_EQ will not work on NaN, we have to test for it directly.
  EXPECT_TRUE(std::isnan(event_a.origin.x));
  EXPECT_TRUE(std::isnan(event_a.origin.y));
  EXPECT_TRUE(std::isnan(event_a.origin.z));
  EXPECT_TRUE(std::isnan(event_a.direction.x));
  EXPECT_TRUE(std::isnan(event_a.direction.y));
  EXPECT_TRUE(std::isnan(event_a.direction.z));

  EXPECT_EQ(event_a.button_state, 0);
  EXPECT_EQ(event_a.hit_node, nullptr);
  EXPECT_EQ(event_a.secondary_hit_node, nullptr);
}

}  // namespace android_xr
