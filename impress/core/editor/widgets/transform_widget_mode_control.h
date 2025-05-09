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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_MODE_CONTROL_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_MODE_CONTROL_H_

#include "core/editor/widgets/transform_widget_mode_control_state.proto.imp.h"
#include "core/ncsb/component.h"

namespace imp::editor {

// Toggles between translate, rotate, and scale modes of the transform widget.
// Attach this component to part of the model you want to click on to toggle
// modes and hook up the parent node of each of the three aspects in the ISF.
class TransformWidgetModeControl : public Component {
 public:
  static constexpr bool kExcludeFromEditor = true;

  void Setup();

  // Cycles through modes.
  void CycleMode();

 private:
  // Whether to control the position, rotation, or scale of the selected node.
  enum class Mode { kTranslate, kRotate, kScale };

  Mode mode_ = Mode::kTranslate;

 private:
  TransformWidgetModeControlState state_;

 public:
  using IsfInfo = IsfInfo<&TransformWidgetModeControl::state_>;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_MODE_CONTROL_H_
