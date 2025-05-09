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

#include "core/proto/parse_message_visitor.h"

namespace imp {

namespace proto {

void ParseMessageVisitor::Accept(HashValue message_type_hash,
                                 void* erased_message) {
  // Finds the registered function (if there is one) for the type of message
  // passed in, and calls the visit functions.
  auto itr = visit_functions_.find(message_type_hash);
  if (itr != visit_functions_.end()) {
    for (auto& visit_function : itr->second) {
      visit_function(erased_message);
    }
  }
}

}  // namespace proto

}  // namespace imp
