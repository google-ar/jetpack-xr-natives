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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_INPUT_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_INPUT_HANDLER_H_

#include <vector>

#include "filament/filament/include/filament/Scene.h"
#include "core/input/pointer_event.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/input/pointer_input_handler.h"
namespace imp::editor {

// EditorInputHandler is a PointerInputHandler that:
// 1. Uses the active camera (Editor or App) to cast rays.
// 2. Uses assigned Dispatcher.
// 3. Sorts nodes (3D widgets and visualizers) in "EditorOverlay" layer (if
// enabled) to the front when building PointerHitEvents.
class EditorInputHandler : public PointerInputHandler {
 public:
  explicit EditorInputHandler(BaseView* view, Dispatcher& dispatcher);

 protected:
  // Casts a ray from a Pointer's screen location out into the view and
  // returns all GenericRayHits. GenericRayhits are sorted first by presence in
  // "EditorOverlay" layer, then by distance.
  std::vector<RayHit> IntersectPointer(const Pointer& p) override;
  std::vector<DoubleRayHit> IntersectPointerPrecise(const Pointer& p) override;

 private:
  filament::Scene* GetEditorOverlayScene() const;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_INPUT_EDITOR_INPUT_HANDLER_H_
