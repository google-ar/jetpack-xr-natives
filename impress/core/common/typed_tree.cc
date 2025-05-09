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

#include "core/common/typed_tree.h"

namespace imp {

void VisitParentThenChildrenStack::Push(size_t parent_index,
                                        size_t num_children_left) {
  stack_.push_back(Item{parent_index, num_children_left});
}

bool VisitParentThenChildrenStack::Empty() const { return stack_.empty(); }

size_t VisitParentThenChildrenStack::ParentForNextIndex() {
  auto& top = stack_.back();
  --top.num_children_left;
  return top.parent_index;
}

void VisitParentThenChildrenStack::FlushEmptyFrames() {
  while (!stack_.back().num_children_left) {
    stack_.pop_back();
    if (stack_.empty()) {
      break;
    }
  }
}

}  // namespace imp
