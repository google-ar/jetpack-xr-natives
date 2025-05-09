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

#include "core/ncsb/component.h"

// This file is named "nc" for "negative compilation" test and will fail to
// compile.

#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/ncsb/node.h"
#include "testing.h"

namespace imp {

class ComponentTest : public testing::ViewFixture {};

class TestNoSetupComponent : public Component {};

class TestSetupNoArgComponent : public Component {
 public:
  void Setup() {}
};

class TestSetupIntArgComponent : public Component {
 public:
  void Setup(int test_value) { test_value_ = test_value; }

  int GetTestValue() { return test_value_; }

 private:
  int test_value_;
};

class UnrelatedComponentA : public Component {};

class UnrelatedComponentB : public Component {};

class TestInvalidSetupReturnTypeComponent : public Component {
 public:
  int Setup() { return 10; }
};

#ifdef TEST_NO_SETUP_COMPONENT
TEST_F(ComponentTest, NoSetupComponent) {
  NodeHandle node = view_->CreateNode();
  node->AddComponent<TestNoSetupComponent>(10);
}
#endif

#ifdef TEST_SETUP_NO_ARG_COMPONENT
TEST_F(ComponentTest, SetupNoArgComponent) {
  NodeHandle node = view_->CreateNode();
  node->AddComponent<TestSetupNoArgComponent>(10);
}
#endif

#ifdef TEST_SETUP_INT_ARG_COMPONENT
TEST_F(ComponentTest, SetupIntArgComponent) {
  NodeHandle node = view_->CreateNode();
  node->AddComponent<TestSetupIntArgComponent>();
}
#endif

#ifdef TEST_CAST_UNRELATED_COMPONENTS
TEST_F(ComponentTest, CastUnrelatedTypes) {
  NodeHandle node = view_->CreateNode();
  ComponentHandle<UnrelatedComponentA> component_a =
      node->AddComponent<UnrelatedComponentA>();
  ComponentHandle<UnrelatedComponentB> component_b =
      static_cast<ComponentHandle<UnrelatedComponentB>>(component_a);
  // Unused
  (void)component_b;
}
#endif

#ifdef TEST_INVALID_SETUP_RETURN_TYPE_COMPONENT
TEST_F(ComponentTest, InvalidSetupReturnTypeComponent) {
  NodeHandle node = view_->CreateNode();
  node->AddComponent<TestInvalidSetupReturnTypeComponent>();
}
#endif

#ifdef TEST_UPDATE_PHASE_DEPENDENCY_MISMATCH_COMPONENT
class UpdatePhaseMismatchParentComponent : public Component {
 public:
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kPreDefault;

  void Update(const FrameTime& frame_time) {}
};

class UpdatePhaseMismatchDepComponent : public Component {
 public:
  using UpdateDependencies = ComponentIds<UpdatePhaseMismatchParentComponent>;

  void Update(const FrameTime& frame_time) {}
};

TEST_F(ComponentTest, TestUpdatePhaseDependencyMismatchComponent) {
  NodeHandle node = view_->CreateNode();
  node->AddComponent<UpdatePhaseMismatchParentComponent>();
  node->AddComponent<UpdatePhaseMismatchDepComponent>();
}
#endif

#ifdef TEST_UPDATE_PHASE_DEPENDENT_MISMATCH_COMPONENT
class UpdatePhaseMismatchParentComponent : public Component {
 public:
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kPreDefault;

  void Update(const FrameTime& frame_time) {}
};

class UpdatePhaseMismatchDepComponent : public Component {
 public:
  using UpdateDependents = ComponentIds<UpdatePhaseMismatchParentComponent>;

  void Update(const FrameTime& frame_time) {}
};

TEST_F(ComponentTest, TestUpdatePhaseDependentsMismatchComponent) {
  NodeHandle node = view_->CreateNode();
  node->AddComponent<UpdatePhaseMismatchParentComponent>();
  node->AddComponent<UpdatePhaseMismatchDepComponent>();
}
#endif

}  // namespace imp
