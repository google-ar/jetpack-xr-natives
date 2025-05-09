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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_STRING_SET_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_STRING_SET_H_

#include <functional>
#include <string>

#include "absl/strings/string_view.h"
#include "core/common/robin_set.h"
#include "core/view/utils/string_hasher.h"

namespace imp {

using StringSet = RobinSet<std::string, StringHasher, std::equal_to<>>;

using StringViewSet =
    RobinSet<absl::string_view, StringHasher, std::equal_to<>>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_STRING_SET_H_
