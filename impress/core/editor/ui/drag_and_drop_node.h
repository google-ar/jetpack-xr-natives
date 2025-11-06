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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_NODE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_NODE_H_

#include <vector>

#include "absl/types/span.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

// Combo ImGui BeginDragAndDropSource + SetDragAndDropPayload for NodeHandles.
bool BeginDragAndDropSource(NodeHandle node);
bool BeginDragAndDropSource(absl::Span<const NodeHandle> nodes);

// Mirror of ImGui::SetDragAndDropPayload for NodeHandles specifically.
void SetDragAndDropPayload(NodeHandle node);
void SetDragAndDropPayload(absl::Span<const NodeHandle> nodes);

// Returns the NodeHandle from the current drag and drop payload if one exists.
NodeHandle GetDragAndDropPayloadNode();
std::vector<NodeHandle> GetDragAndDropPayloadNodes();

// Mirror of ImGui::AcceptDragAndDropPayload that returns NodeHandles.
NodeHandle AcceptDragAndDropPayloadNode();
std::vector<NodeHandle> AcceptDragAndDropPayloadNodes();

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_DRAG_AND_DROP_NODE_H_
