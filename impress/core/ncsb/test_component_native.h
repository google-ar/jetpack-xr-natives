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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_TEST_COMPONENT_NATIVE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_TEST_COMPONENT_NATIVE_H_

#include "core/ncsb/isf_info.h"
namespace imp {
namespace test {

struct DependentA {
  static constexpr absl::string_view kType = "imp.test.DependentA";
  using IsfInfo = StatelessIsfInfo<DependentA, kType>;
};

struct TypeWithDependencyA {
  using IsfDependencies = IsfDependencies<DependentA>;
};

struct DependentB {
  static constexpr absl::string_view kType = "imp.test.DependentB";
  using IsfInfo = StatelessIsfInfo<DependentB, kType>;
};

struct TypeWithDependencyB {
  using IsfDependencies = IsfDependencies<DependentB>;
};

}  // namespace test
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_TEST_COMPONENT_NATIVE_H_
