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

#include "core/assets/gltf/object_model/property_pointer.h"

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/assets/gltf/object_model/token_parser.h"
#include "core/ncsb/node_handle.h"

namespace imp::gltf {

using PointerValue = PropertyPointer::PointerValue;

PropertyPointer::PropertyPointer(const PointerDeclaration* pointer_declaration,
                                 const std::vector<ParsedToken>& parsed_tokens)
    : pointer_declaration_(pointer_declaration), parsed_tokens_(parsed_tokens) {
  parsed_tokens_ = parsed_tokens;
}

absl::StatusOr<PointerValue> PropertyPointer::GetValue(
    NodeHandle gltf_model) const {
  return pointer_declaration_->GetValue(gltf_model, parsed_tokens_);
}

absl::Status PropertyPointer::SetValue(NodeHandle gltf_model,
                                       PointerValue value) const {
  return pointer_declaration_->SetValue(gltf_model, parsed_tokens_, value);
}

}  // namespace imp::gltf
