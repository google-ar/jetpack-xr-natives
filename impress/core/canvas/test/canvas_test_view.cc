// Copyright 2026 Google LLC
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
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/canvas/assets/canvas_scuba_test_assets.h"
#include "core/canvas/canvas_source.h"
#include "core/canvas/constants.h"
#include "core/canvas/scoped_canvas.h"
#include "core/render/primitive_shape_renderer.h"
#include "core/render/primitive_shape_renderer_state.proto.imp.h"
#include "core/render/texture.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "imp.h"

namespace imp {
namespace canvas_test {

class CanvasTestView : public imp::View {
 protected:
  void Setup() override {
    canvas_source_ = imp::CanvasSource::Create(GetContext());

    node_ = CreateNode();
    node_->SetLocalPosition({0.0f, 0.0f, -5.0f});

    // Setup a quad to show the texture.
    // We use a simple material that takes a single "BaseColor" texture.
    imp::PrimitiveShapeRendererState primitive_shape_state;
    primitive_shape_state.primitive = {
        .material =
            imp::MaterialDefinition{
                .asset = std::string(
                    canvas_scuba_test_assets::kQuadCmat.GetIdentifier()),
            },
        .mesh = imp::PrimitiveShapeRendererState::QuadMesh{}};

    node_
        ->AddComponentWithState<imp::PrimitiveShapeRenderer>(
            primitive_shape_state)
        .KeptBy(this);
  }

  void Update(const imp::FrameTime& frame_time) override {
    auto renderer = node_->GetComponent<imp::PrimitiveShapeRenderer>();
    if (!renderer || !renderer->GetMaterial()) {
      return;
    }

    const uint2 canvas_size = {1024, 1024};
    std::unique_ptr<imp::ScopedCanvas> canvas =
        canvas_source_->StartDrawing(*this, canvas_size);

    if (canvas->DidTextureChange()) {
      renderer->GetMaterial()->SetParameter("BaseColor", canvas->GetTexture());
    }

    canvas->DrawColor(float4(0.1f, 0.1f, 0.2f, 1.0f));

    // 1. Validate DrawText
    const imp::ScopedCanvas::TextOptions title_options{
        .size_pixels = 80,
        .horizontal_alignment = imp::TextHorizontalAlignment::kCenter,
        .vertical_alignment = imp::TextVerticalAlignment::kTopExtent,
        .color = {1.0f, 1.0f, 1.0f, 1.0f}};
    canvas->DrawText("ScopedCanvas Validation", {512, 50}, title_options);

    const imp::ScopedCanvas::TextOptions label_options{
        .size_pixels = 50, .color = {0.8f, 0.8f, 1.0f, 1.0f}};

    canvas->DrawText("DrawText Left", {50, 200}, label_options);

    imp::ScopedCanvas::TextOptions centered_options = label_options;
    centered_options.horizontal_alignment =
        imp::TextHorizontalAlignment::kCenter;
    canvas->DrawText("DrawText Center", {512, 300}, centered_options);

    imp::ScopedCanvas::TextOptions right_options = label_options;
    right_options.horizontal_alignment = imp::TextHorizontalAlignment::kRight;
    canvas->DrawText("DrawText Right", {974, 400}, right_options);

    // 2. Validate DrawGlyph
    canvas->DrawText("DrawGlyph Row:", {50, 550}, label_options);

    const imp::ScopedCanvas::TextOptions glyph_options{
        .size_pixels = 100, .color = {1.0f, 0.5f, 0.0f, 1.0f}};

    std::string test_chars = "ABC!@#";
    std::vector<imp::ScopedCanvas::GlyphAdvance> glyphs =
        canvas_source_->GetTextGlyphs(test_chars, glyph_options);

    float2 glyph_pos = {100, 700};
    for (const auto& advance : glyphs) {
      canvas->DrawGlyph(advance.glyph, glyph_pos, glyph_options);
      glyph_pos.x += advance.width + 20;
    }

    // 3. Validate Stroke
    imp::ScopedCanvas::TextOptions stroke_options = label_options;
    stroke_options.size_pixels = 100;
    stroke_options.stroke_width_pixels = 10;
    stroke_options.stroke_color = {1.0f, 0.0f, 0.0f, 1.0f};
    stroke_options.horizontal_alignment = imp::TextHorizontalAlignment::kCenter;
    canvas->DrawText("STROKE TEXT", {512, 900}, stroke_options);
  }

 private:
  std::unique_ptr<imp::CanvasSource> canvas_source_;
  imp::NodeHandle node_;
};

const bool kIsCreateViewAssigned =
    imp::client_api::SetCreateViewFn([](absl::string_view identifier) {
      return imp::View::Create<CanvasTestView>("CanvasTest");
    });

}  // namespace canvas_test
}  // namespace imp
