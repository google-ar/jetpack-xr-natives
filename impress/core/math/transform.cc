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

#include "core/math/transform.h"

#include "absl/strings/str_format.h"
#include "core/common/string_helpers.h"
#include "core/math/quat.h"
#include "core/math/vec.h"

namespace imp {

std::string ToString(const Transform<float>& trs) {
  return absl::StrFormat("<T=%s R=%s S=%s>", ToString(trs.translation).c_str(),
                         ToString(trs.rotation).c_str(),
                         ToString(trs.scale).c_str());
}

}  // namespace imp
