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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_RECIPE_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_RECIPE_EVENT_H_

#include <string>

#include "absl/strings/string_view.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp {

// Event sent to the dispatcher when an event in the Recipe Graph is fired.
class RecipeEvent : public Event {
 public:
  RecipeEvent(absl::string_view name, const recipe::Variables& params)
      : name_(std::string(name)), params_(params) {}

  // Returns the value of the given key, or nullptr if not found.
  const recipe::Variable* GetParam(absl::string_view key) const;

  absl::string_view GetName() const { return name_; }

 private:
  std::string name_;
  const recipe::Variables& params_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_RECIPE_EVENT_H_
