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

#include "core/resources/resource_definition.h"

namespace imp {
namespace resources {

absl::string_view ResourceDefinition::GetIdentifier() const {
  if (register_resource_fn_) {
    register_resource_fn_();
  }

  return identifier_;
}

absl::string_view ResourceDefinition::GetUrl() const {
  if (register_resource_fn_) {
    register_resource_fn_();
  }

  return url_;
}

}  // namespace resources
}  // namespace imp
