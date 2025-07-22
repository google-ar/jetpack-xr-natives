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

#include "core/view/framework/gestures/gesture_manager.h"

#include "core/view/framework/gestures/double_tap_gesture.h"
#include "core/view/framework/gestures/drag_gesture.h"
#include "core/view/framework/gestures/hover_gesture.h"
#include "core/view/framework/gestures/multi_drag_gesture.h"
#include "core/view/framework/gestures/pinch_gesture.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/framework/gestures/twist_gesture.h"

namespace imp {

GestureManager::GestureManager(Dispatcher* dispatcher)
    : dispatcher_(dispatcher),
      pointer_utils_(),
      recognizers_(),
      recognizer_hashes_() {
  // Add default gestures for recognition by GestureManager. The defaults can be
  // removed by calling RemoveGestureRecognizer on the GestureManager.
  AddGestureRecognizer<PinchGesture>();
  AddGestureRecognizer<TwistGesture>();
  AddGestureRecognizer<MultiDragGesture>();
  AddGestureRecognizer<DragGesture>();
  AddGestureRecognizer<DoubleTapGesture>();
  AddGestureRecognizer<TapGesture>();
  AddGestureRecognizer<HoverGesture>();
}

void GestureManager::OnPointerHitEvent(const PointerHitEvent& hit_event) {
  for (auto const& recognizer : recognizers_) {
    recognizer->OnPointerHitEvent(hit_event);
  }
}

}  // namespace imp
