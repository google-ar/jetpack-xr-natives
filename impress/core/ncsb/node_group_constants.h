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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_GROUP_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_GROUP_CONSTANTS_H_

#include "absl/strings/string_view.h"
#include "core/common/hash.h"

namespace imp {

// The name of the group representing the main render pass that
// outputs to the surface each frame.
//
// All nodes are part of this group by default. If a node is
// removed from this group, it won't be visible on the main surface.
//
// The name is "Main".
static constexpr absl::string_view kMainGroupName = "Main";
static constexpr HashValue kMainGroupHash = Hash(kMainGroupName);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_GROUP_CONSTANTS_H_
