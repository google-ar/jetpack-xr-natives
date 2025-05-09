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

#include "core/recipes/recipe_event.h"

#include "absl/strings/string_view.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp {
const recipe::Variable* RecipeEvent::GetParam(absl::string_view key) const {
  auto itr = params_.find(key);
  if (itr != params_.end()) {
    return &itr.value();
  }

  return nullptr;
}
}  // namespace imp
