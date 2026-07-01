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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SEARCH_FILTER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SEARCH_FILTER_H_

#include "absl/strings/string_view.h"
#include "core/editor/widgets/performance/sample_processor_types.h"

namespace imp::editor {

// Returns true if the query is empty or if the sample name matches the query.
bool MatchesSearchQuery(absl::string_view name, absl::string_view query);

// Filters a tree of sample nodes in place, retaining only nodes that match
// the query or have descendants/children that do. Returns true if the node or
// any of its descendants match the query.
bool FilterTree(SampleNode* node, absl::string_view query);

// Checks if a node has any descendant with the specified name.
bool HasDescendantWithName(SampleNode* node, absl::string_view name);

// Checks if the given node is an ancestor of the selected sample.
bool IsAncestorOfSelected(SampleNode* node, absl::string_view selected_name);

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_PERFORMANCE_SEARCH_FILTER_H_
