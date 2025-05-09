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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_RENDER_SETTING_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_RENDER_SETTING_UTILS_H_

#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/View.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"

namespace imp {

using render_settings::OverrideMode;
using render_settings::ViewRenderSettings;

// Configure a filament::View's render settings with overrides.
// override_settings will be applied on top of the base values. Most fields are
// optional in ViewRenderSettings, meaning that if a value is present, it will
// override the corresponding base value and if it's not present, the base value
// will be used.
// If override_mode is OVERRIDE_MODE_OVERRIDE_CURRENT, then the current values
// in the target filament::View will be used as the base values.
// If override_mode is OVERRIDE_MODE_OVERRIDE_FROM_SOURCE, then the source
// filament::View's current settings will be used as the base values.
// If override_settings is nullptr, then the base values will be used and no
// overrides will be applied.
void ConfigureViewRenderSettingsWithOverrides(
    const OverrideMode override_mode, filament::View* target,
    const filament::View* source, const ViewRenderSettings* override_settings,
    filament::Engine* engine);

// Override render settings of an existing filament::View.
void OverrideViewRenderSettings(filament::View* target,
                                const ViewRenderSettings* override_settings,
                                filament::Engine* engine);

// Returns a ViewRenderSettings of the given filament::View. Note that this
// doesn't return the color grading options.
ViewRenderSettings GetViewRenderSettings(const filament::View* view);
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_VIEW_UTILS_H_
