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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_FACTORY_H_

#include <memory>

#include "core/canvas/async_canvas_source.h"
#include "core/common/context.h"

namespace imp {

namespace AsyncCanvasSourceFactory {

std::unique_ptr<AsyncCanvasSource> Create(
    Context context, bool use_hardware_rendering = true,
    bool force_auto_method_rendering = false,
    bool force_individual_glyph_source_instances = false,
    bool enable_label_prep_profile_logging = false,
    bool use_bitmap_surface_provider = false);

}  // namespace AsyncCanvasSourceFactory

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_FACTORY_H_
