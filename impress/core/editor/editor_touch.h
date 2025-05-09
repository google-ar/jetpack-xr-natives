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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_TOUCH_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_TOUCH_H_

#include "core/common/invocable.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

using UndoEditorTouchFn = Invocable<void()>;

// Indicates that the node has been modified by the editor so that it can be
// properly saved as authored content.
//
// Returns a function that can be called to undo the touch, which is a no-op if
// the node was already tracked prior to being touched.
UndoEditorTouchFn EditorTouch(NodeHandle node);

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_TOUCH_H_
