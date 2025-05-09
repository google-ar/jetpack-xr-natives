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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_VISUALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_VISUALIZER_H_

#include "core/ncsb/component.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// Visualizer is 2D/3D models attached to invisible targets, e.g. camera and
// light, to give the users a better idea of the status of those targets
// in develop mode.
class Visualizer : public imp::Component {
 public:
  static constexpr bool kExcludeFromEditor = true;
  static constexpr UpdateMode kUpdateMode = UpdateMode::kAlwaysUpdate;

  virtual ~Visualizer() = default;
  virtual void Setup(NodeHandle target) = 0;
  virtual void Update(const FrameTime& frame_time) = 0;
  NodeHandle GetTarget() const { return target_; }

 protected:
  NodeHandle target_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_VISUALIZER_H_
