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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_INPUT_SETTINGS_WIDGET_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_INPUT_SETTINGS_WIDGET_CONSTANTS_H_

#include "absl/strings/string_view.h"

namespace imp::editor {

inline constexpr absl::string_view kInvertCameraYKey =
    "previewer_invert_camera_y";
inline constexpr absl::string_view kInvertMouseScrollKey =
    "previewer_invert_mouse_scroll";
inline constexpr absl::string_view kUseLegacyCameraControlsKey =
    "editor_use_legacy_camera_controls";
inline constexpr bool kUseLegacyCameraControlsDefault = true;

#ifdef IMP_INVERT_EDITOR_INPUT
inline constexpr bool kInvertCameraYDefault = true;
inline constexpr bool kInvertMouseScrollDefault = true;
#else
inline constexpr bool kInvertCameraYDefault = false;
inline constexpr bool kInvertMouseScrollDefault = false;
#endif

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_INPUT_SETTINGS_WIDGET_CONSTANTS_H_
