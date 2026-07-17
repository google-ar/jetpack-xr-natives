// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "absl/memory/memory.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/platform_canvas_source.h"
#include "core/common/context.h"
#include "core/config.h"
#if IMP_PLATFORM(WASM)
#include "core/canvas/wasm_async_canvas_source.h"
#else
#include "core/canvas/async_canvas_source_wrapper.h"
#include "core/canvas/canvas_source.h"
#endif

namespace imp {

namespace AsyncCanvasSourceFactory {

std::unique_ptr<AsyncCanvasSource> Create(
    Context context, bool use_hardware_rendering,
    bool force_auto_method_rendering,
    bool force_individual_glyph_source_instances,
    bool enable_label_prep_profile_logging, bool use_bitmap_surface_provider) {
#if IMP_PLATFORM(WASM)
  return std::make_unique<WasmAsyncCanvasSource>(
      enable_label_prep_profile_logging);
#else
  return absl::make_unique<AsyncCanvasSourceWrapper>(CanvasSource::Create(
      context, use_hardware_rendering, force_auto_method_rendering,
      PlatformCanvasSource::kDefaultGlyphCacheSizeBytes,
      force_individual_glyph_source_instances, use_bitmap_surface_provider));
#endif
};

}  // namespace AsyncCanvasSourceFactory

}  // namespace imp
