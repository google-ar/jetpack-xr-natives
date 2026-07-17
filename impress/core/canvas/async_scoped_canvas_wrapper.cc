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

#include "core/canvas/async_scoped_canvas_wrapper.h"

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"

namespace imp {

Texture* AsyncScopedCanvasWrapper::GetTexture() {
  return scoped_canvas_->GetTexture();
}

bool AsyncScopedCanvasWrapper::DidTextureChange() const {
  return scoped_canvas_->DidTextureChange();
}

void AsyncScopedCanvasWrapper::DrawColor(float3 color) {
  return scoped_canvas_->DrawColor(color);
}

void AsyncScopedCanvasWrapper::DrawColor(float4 color) {
  return scoped_canvas_->DrawColor(color);
}

void AsyncScopedCanvasWrapper::DrawRoundedRect(float3 color,
                                               float2 corner_radius,
                                               const Rect& rect) {
  return scoped_canvas_->DrawRoundedRect(color, corner_radius, rect);
}

void AsyncScopedCanvasWrapper::DrawRoundedRect(float4 color,
                                               float2 corner_radius,
                                               const Rect& rect) {
  return scoped_canvas_->DrawRoundedRect(color, corner_radius, rect);
}

void AsyncScopedCanvasWrapper::DrawText(absl::string_view text, float2 pos,
                                        const TextOptions& text_options) {
  return scoped_canvas_->DrawText(text, pos, text_options);
}

void AsyncScopedCanvasWrapper::DrawGlyph(
    GlyphId glyph, float2 pos, const TextOptions& text_options,
    const TextMetrics* pre_cached_metrics) {
  return scoped_canvas_->DrawGlyph(glyph, pos, text_options,
                                   pre_cached_metrics);
};

void AsyncScopedCanvasWrapper::ClearRect(const Rect& rect) {
  return scoped_canvas_->ClearRect(rect);
}

Future<absl::Status> AsyncScopedCanvasWrapper::PrepareToUpdateTexture() {
  return Future<absl::Status>(absl::OkStatus());
};

bool AsyncScopedCanvasWrapper::SupportsSynchronousTextureUpdate() const {
  return true;
}

}  // namespace imp
