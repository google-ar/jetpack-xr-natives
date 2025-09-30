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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_CONSTANTS_H_

#include "absl/strings/string_view.h"
#include "core/editor/layout/layout_config.proto.imp.h"

namespace imp::editor {
// LINT.IfChange(overlay_objects_group_name)
static constexpr absl::string_view kOverlayGroup = "EditorOverlay";
// LINT.ThenChange(//depot/google3/third_party/impress/core/editor/widgets/transform_widget.textproto)

static constexpr absl::string_view kNodeWidgetHeaderName = "Node";

static constexpr LayoutConfig kDefaultMobileLayoutConfig{
    .layout_type = LayoutConfig::LayoutType::SINGLE_TABBED_WINDOW,
    .initial_tabbed_window_state = {
        .pin_state = LayoutConfig::WindowPinState::PINNED_TO_BOTTOM_DEFAULT,
        .expanded_state = LayoutConfig::WindowExpandedState::EXPANDED,
        .max_window_height_multiplier = 0.35f}};

static constexpr LayoutConfig kDefaultDesktopLayoutConfig{
    .layout_type = LayoutConfig::LayoutType::MULTIPLE_WINDOWS_DEFAULT,
    .initial_tabbed_window_state = {
        .pin_state = LayoutConfig::WindowPinState::PINNED_TO_BOTTOM_DEFAULT,
        .expanded_state = LayoutConfig::WindowExpandedState::COLLAPSED_DEFAULT,
        .max_window_height_multiplier = 1.0f}};

static constexpr LayoutConfig kDefaultXrLayoutConfig{
    .layout_type = LayoutConfig::LayoutType::MULTIPLE_WINDOWS_WORLD_LAYOUT,
    .initial_tabbed_window_state = {
        .pin_state = LayoutConfig::WindowPinState::PINNED_TO_TOP,
        .expanded_state = LayoutConfig::WindowExpandedState::COLLAPSED_DEFAULT,
        .max_window_height_multiplier = 1.0f}};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_EDITOR_CONSTANTS_H_
