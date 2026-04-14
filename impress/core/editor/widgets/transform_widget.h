/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_H_

#include <vector>

#include "absl/strings/string_view.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_phase.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// Controls the position of the transform widget and whether it is enabled.
// This component should be attached to the main widget node in the ISF.
class TransformWidget : public imp::Component {
 public:
  static constexpr bool kExcludeFromEditor = true;

  void Setup();
  void Update(const FrameTime& frame_time);

  static constexpr UpdateMode kUpdateMode = UpdateMode::kAlwaysUpdate;
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;

 private:
  static constexpr absl::string_view kType = "imp.editor.TransformWidget";

  std::vector<NodeHandle> active_nodes_;

 public:
  using IsfInfo = StatelessIsfInfo<TransformWidget, kType>;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_H_
