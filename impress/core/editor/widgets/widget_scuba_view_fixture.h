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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_WIDGET_SCUBA_VIEW_FIXTURE_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_WIDGET_SCUBA_VIEW_FIXTURE_H_

#include <memory>

#include "core/editor/editor_constants.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/layout/layout_config.proto.imp.h"
#include "core/editor/widget_ui_system.h"
#include "testing/imgui_scuba_view_fixture.h"

namespace imp::editor {

// A version of ImGuiScubaViewFixture with a WidgetUiSystem for testing Widgets.
class WidgetScubaViewFixture : public imp::testing::ImGuiScubaViewFixture {
 public:
  WidgetScubaViewFixture(
      LayoutConfig layout_config = kDefaultDesktopLayoutConfig)
      : imp::testing::ImGuiScubaViewFixture(
            "third_party/impress/core/editor/widgets/scuba_goldens"),
        widget_ui_system_(view_, true,
                          std::make_unique<LayoutComposer>(layout_config)) {}

 protected:
  WidgetUiSystem widget_ui_system_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_WIDGET_SCUBA_VIEW_FIXTURE_H_
