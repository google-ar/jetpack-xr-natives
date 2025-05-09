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

#include "core/proto/proto_matcher_utils.h"

namespace imp {
namespace testing {
namespace proto {
namespace matcher_utils {

StringMap<DebugJsonWriter::VisitRegisteredAnyFn>*
DebugJsonWriter::GetVisitRegisteredFnMap() {
  // It's best to store static data structures as a static pointer inside of
  // a static function, per (broken link) and (broken link).
  static StringMap<DebugJsonWriter::VisitRegisteredAnyFn>*
      visit_registered_fn_map =
          new StringMap<DebugJsonWriter::VisitRegisteredAnyFn>();
  return visit_registered_fn_map;
}

}  // namespace matcher_utils
}  // namespace proto
}  // namespace testing
}  // namespace imp
