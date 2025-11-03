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

#include "core/canvas/canvas_source.h"

#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/canvas/platform_canvas_source.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"

#if IMP_PLATFORM(ANDROID)
#include "core/canvas/android_platform_canvas_source.h"
#elif IMP_PLATFORM(IOS)
#include "core/canvas/ios_platform_canvas_source.h"
#elif IMP_PLATFORM(WASM)
#include "core/canvas/wasm_platform_canvas_source.h"
#else
#include "core/canvas/desktop_platform_canvas_source.h"
#endif

namespace imp {

std::unique_ptr<CanvasSource> CanvasSource::Create(
    Context context, bool use_hardware_rendering) {
  // Instantiate the correct platform implementation.
#if IMP_PLATFORM(ANDROID)
  // TODO: Shaper method fails to render text weight properly.
  // Change to kAuto once this bug is fixed.
  auto platform_source = std::make_unique<AndroidPlatformCanvasSource>(
      context, AndroidGlyphSource::Method::kPath, use_hardware_rendering);
#elif IMP_PLATFORM(IOS)
  auto platform_source = std::make_unique<IosPlatformCanvasSource>();
#elif IMP_PLATFORM(WASM)
  auto platform_source = std::make_unique<WasmPlatformCanvasSource>();
#else
  auto platform_source = std::make_unique<DesktopPlatformCanvasSource>();
#endif

  return absl::WrapUnique(new CanvasSource(std::move(platform_source)));
}

bool CanvasSource::IsFeatureSupported(ScopedCanvas::Feature feature) {
  absl::MutexLock lock(platform_source_mutex_);
  return platform_source_->IsFeatureSupported(feature);
}

CanvasSource::CanvasSource(
    std::unique_ptr<PlatformCanvasSource> platform_source)
    : platform_source_(std::move(platform_source)) {}

Future<absl::Status> CanvasSource::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(platform_source_mutex_);
  return platform_source_->PrepareFont(text, text_options);
}

ScopedCanvas::TextMetrics CanvasSource::GetTextMetrics(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(platform_source_mutex_);
  return platform_source_->GetTextMetrics(text, text_options);
}

ScopedCanvas::TextMetrics CanvasSource::GetGlyphMetrics(
    ScopedCanvas::GlyphId glyph,
    const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(platform_source_mutex_);
  return platform_source_->GetGlyphMetrics(glyph, text_options);
}

std::vector<ScopedCanvas::GlyphGroup> CanvasSource::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(platform_source_mutex_);
  return platform_source_->GetCombinedCharacterGroups(text, text_options);
}

std::vector<float> CanvasSource::GetTextWidths(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(platform_source_mutex_);
  return platform_source_->GetTextWidths(text, text_options);
}

std::vector<ScopedCanvas::GlyphAdvance> CanvasSource::GetTextGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(platform_source_mutex_);
  return platform_source_->GetTextGlyphs(text, text_options);
}

ScopedCanvas::FontInfo CanvasSource::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(platform_source_mutex_);
  return platform_source_->GetFontInfo(text_options);
}

std::unique_ptr<ScopedCanvas> CanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) {
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "StartDrawing may not be called on the foreground executor.";
  }
  absl::MutexLock lock(platform_source_mutex_);
  std::unique_ptr<ScopedCanvas> scoped_canvas =
      platform_source_->StartDrawing(view, pixel_size, draw_mode);
  return scoped_canvas;
}

std::unique_ptr<ScopedCanvas> CanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "StartDrawing may not be called on the foreground executor.";
  }
  absl::MutexLock lock(platform_source_mutex_);
  std::unique_ptr<ScopedCanvas> scoped_canvas = platform_source_->StartDrawing(
      view, pixel_size, std::move(on_texture_changed_fn), draw_mode, loc);
  return scoped_canvas;
}

void CanvasSource::ForceReset() {
  absl::MutexLock lock(platform_source_mutex_);
  platform_source_->ForceReset();
}

}  // namespace imp
