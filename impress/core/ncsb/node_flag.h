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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_FLAG_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_FLAG_H_

#include "core/common/bit_flag.h"

namespace imp {
using NodeFlag = BitFlag;

enum NodeFlags : NodeFlag {
  // No flags set.
  kInvalid = 0,
  // All possible flags set.
  kAll = ~static_cast<BitFlag>(0),
  // See Node::IsEnabled()
  kIsEnabled = 1 << 0,
  // See Node::IsActive()
  kIsActive = 1 << 1,
  // This flag will be set for nodes with no parent.
  kIsRoot = 1 << 2,
  // This flag will be set for nodes with their group overridden.
  kIsGroupsOverridden = 1 << 3,
  kIsEditorStaging = 1 << 4,
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_FLAG_H_
