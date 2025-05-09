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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_SCOPED_CANVAS_WRAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_SCOPED_CANVAS_WRAPPER_H_

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"

namespace imp {

// A wrapper around a synchronous ScopedCanvas to make it implement the
// AsyncScopedCanvas interface. This simply delegates all calls to the
// synchronous ScopedCanvas and returns an immediately successful future
// on call to WaitForTextureToUpdate.
struct AsyncScopedCanvasWrapper : public AsyncScopedCanvas {
  explicit AsyncScopedCanvasWrapper(std::unique_ptr<ScopedCanvas> scoped_canvas)
      : scoped_canvas_(std::move(scoped_canvas)) {}

  ~AsyncScopedCanvasWrapper() = default;

  Texture* GetTexture();
  bool DidTextureChange() const;
  void DrawColor(float3 color);
  void DrawColor(float4 color);
  void DrawRoundedRect(float3 color, float2 corner_radius, const Rect& rect);
  void DrawRoundedRect(float4 color, float2 corner_radius, const Rect& rect);
  void DrawText(absl::string_view text, float2 pos,
                const TextOptions& text_options);
  void DrawGlyph(GlyphId glyph, float2 pos, const TextOptions& text_options);
  void ClearRect(const Rect& rect);
  Future<absl::Status> PrepareToUpdateTexture();
  bool SupportsSynchronousTextureUpdate() const;

 private:
  std::unique_ptr<ScopedCanvas> scoped_canvas_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_SCOPED_CANVAS_WRAPPER_H_
