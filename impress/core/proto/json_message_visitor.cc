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

#include "core/proto/json_message_visitor.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/hash.h"

namespace imp::proto {

absl::Status JsonMessageVisitor::Poll(HashValue message_type_hash,
                                      void* erased_message,
                                      void* erased_visitor,
                                      const char* cursor) {
  // Finds the registered function (if there is one) for the type of message
  // passed in, and calls the pre visit functions.
  auto itr = pre_visit_functions_.find(message_type_hash);
  if (itr != pre_visit_functions_.end()) {
    for (auto& pre_visit_function : itr->second) {
      absl::Status status =
          pre_visit_function(erased_message, erased_visitor, cursor);
      if (!status.ok()) {
        return status;
      }
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<bool> JsonMessageVisitor::Handle(
    HashValue message_type_hash, void* erased_message, int field_id,
    void* erased_visitor, const char* cursor, int token_type) {
  // Finds the registered function (if there is one) for the type of message
  // passed in, and calls the visit functions.
  auto itr = visit_functions_.find(message_type_hash);
  if (itr != visit_functions_.end()) {
    absl::StatusOr<bool> visit_result = itr->second(
        erased_message, field_id, erased_visitor, cursor, token_type);
    return visit_result;
  }

  return false;
}

void JsonMessageVisitor::Accept(HashValue message_type_hash,
                                void* erased_message) {
  // Finds the registered function (if there is one) for the type of message
  // passed in, and calls the post visit functions.
  auto itr = post_visit_functions_.find(message_type_hash);
  if (itr != post_visit_functions_.end()) {
    for (auto& post_visit_function : itr->second) {
      post_visit_function(erased_message);
    }
  }
}

}  // namespace imp::proto
