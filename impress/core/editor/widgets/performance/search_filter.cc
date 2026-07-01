// Copyright 2026 Google LLC
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

#include "core/editor/widgets/performance/search_filter.h"

#include "absl/strings/match.h"
#include "absl/strings/string_view.h"
#include "core/editor/widgets/performance/sample_processor_types.h"

namespace imp::editor {

bool MatchesSearchQuery(const absl::string_view name,
                        const absl::string_view query) {
  if (query.empty()) return true;
  return absl::StrContainsIgnoreCase(name, query);
}

bool FilterTree(SampleNode* node, const absl::string_view query) {
  if (!node) return false;

  const bool matches = MatchesSearchQuery(node->result->GetName(), query);

  SampleNode* prev_child = nullptr;
  SampleNode* child = node->first_child;
  bool any_child_matches = false;

  while (child) {
    SampleNode* next_child = child->next_sibling;

    if (FilterTree(child, query)) {
      any_child_matches = true;

      if (prev_child == nullptr) {
        node->first_child = child;
      } else {
        prev_child->next_sibling = child;
      }

      prev_child = child;
    }
    child = next_child;
  }

  if (prev_child) {
    prev_child->next_sibling = nullptr;
  } else {
    node->first_child = nullptr;
  }

  return matches || any_child_matches;
}

bool HasDescendantWithName(SampleNode* node, const absl::string_view name) {
  if (!node || name.empty()) return false;

  SampleNode* child = node->first_child;

  while (child) {
    if (child->result->GetName() == name) return true;

    if (HasDescendantWithName(child, name)) return true;

    child = child->next_sibling;
  }
  return false;
}

bool IsAncestorOfSelected(SampleNode* node,
                          const absl::string_view selected_name) {
  if (selected_name.empty()) return false;
  return HasDescendantWithName(node, selected_name);
}

}  // namespace imp::editor
