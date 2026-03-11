/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_MODE_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_MODE_H_

namespace svxr {

// Holds data on the different interaction modes that are currently engaged.
class InteractionMode {
 public:
  enum class SelectedMode { kUnselected, kSelected };
  enum class PointerMode { kNothing, kHover, kPress };
  enum class TransformMode { kNothing, kTranslate, kRotate, kScale };
  enum class ActiveMode { kNothing, kInteracting };
  enum class GazeMode { kDefault, kGaze };

  void SetSelected(SelectedMode selected) { selected_ = selected; }
  void SetPointer(PointerMode pointer) { pointer_ = pointer; }
  void SetTransform(TransformMode transform) { transform_ = transform; }
  void SetActive(ActiveMode active) { active_ = active; }
  void SetGaze(GazeMode gaze) { gaze_ = gaze; }

  bool TestSelected(SelectedMode test_mode) const {
    return selected_ == test_mode;
  }
  bool TestPointer(PointerMode test_mode) const {
    return pointer_ == test_mode;
  }
  bool TestTransform(TransformMode test_mode) const {
    return transform_ == test_mode;
  }
  bool TestActive(ActiveMode test_mode) const { return active_ == test_mode; }
  bool TestGaze(GazeMode test_mode) const { return gaze_ == test_mode; }

  InteractionMode() = default;

  bool ToggleSelect() {
    selected_ = (selected_ == SelectedMode::kSelected)
                    ? SelectedMode::kUnselected
                    : SelectedMode::kSelected;
    return selected_ == SelectedMode::kSelected;
  }

 private:
  SelectedMode selected_ = SelectedMode::kSelected;
  PointerMode pointer_ = PointerMode::kNothing;
  TransformMode transform_ = TransformMode::kNothing;
  ActiveMode active_ = ActiveMode::kNothing;
  GazeMode gaze_ = GazeMode::kGaze;
};

}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_MODE_H_
