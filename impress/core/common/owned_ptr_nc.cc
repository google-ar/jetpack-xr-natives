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

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "mediapipe/framework/port/status_matchers.h"
#include "core/common/owned_ptr.h"

// This file is named "nc" for "negative compilation" test and will fail to
// compile.

namespace imp {

class TestType {
 public:
  TestType(int val) : val_(val) {}

  int GetVal() const { return val_; }

 private:
  int val_;
};

struct TestTypeWithBase : public TestType {
  TestTypeWithBase(int val) : TestType(val) {}
};

#ifdef TEST_IMPLICIT_DOWNCAST_COPY
TEST(OwnedPtrTest, CanDowncastBorrowedPtr) {
  OwnedPtr<TestTypeWithBase> owned = MakeOwned<TestTypeWithBase>(5);
  BorrowedPtr<TestType> upcasted = owned.Borrow();

  // Should not compile, implicit downcasting is not allowed.
  BorrowedPtr<TestTypeWithBase> downcasted = upcasted;
}
#endif

#ifdef TEST_IMPLICIT_DOWNCAST_MOVE
TEST(OwnedPtrTest, CanDowncastMovedBorrowedPtr) {
  OwnedPtr<TestTypeWithBase> owned = MakeOwned<TestTypeWithBase>(5);
  BorrowedPtr<TestType> upcasted = owned.Borrow();

  // Should not compile, implicit downcasting is not allowed.
  BorrowedPtr<TestTypeWithBase> downcasted = std::move(upcasted);
}
#endif

}  // namespace imp
