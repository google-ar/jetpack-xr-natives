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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_DEFAULT_VIEW_CONFIG_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_DEFAULT_VIEW_CONFIG_H_

#include "core/view/utils/proto/view_config.proto.imp.h"

namespace imp {

constexpr float kForegroundExecutorTimeoutMs = 4.0f;
constexpr float kBackgroundExecutorTimeoutMs = 1.0f;

constexpr ViewConfig kDefaultViewConfig = {
    .shader_caching_mode =
        ViewConfig::ShaderCachingMode::SHADER_CACHING_MODE_DISABLED,
    .default_lighting_loading =
        ViewConfig::DefaultLightingLoading::DEFAULT_LIGHTING_LOADING_ENABLED,
    .foreground_executor_timeout_ms = kForegroundExecutorTimeoutMs,
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_UTILS_DEFAULT_VIEW_CONFIG_H_
