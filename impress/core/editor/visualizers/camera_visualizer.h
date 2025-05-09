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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_CAMERA_VISUALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_CAMERA_VISUALIZER_H_

#include "core/editor/visualizers/visualizer.h"

namespace imp::editor {

// Controls the position and rotation of camera visualizers
// and whether it is enabled.
class CameraVisualizer : public Visualizer {
 public:
  void Setup(NodeHandle camera) override;
  void Update(const FrameTime& frame_time) override;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_CAMERA_VISUALIZER_H_
