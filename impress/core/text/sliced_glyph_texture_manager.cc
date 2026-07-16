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

#include "core/text/sliced_glyph_texture_manager.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>

#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/ColorGrading.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/ToneMapper.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/filament/include/filament/View.h"
#include "filament/filament/include/filament/Viewport.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/EntityManager.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/text/glyph_atlas_slice.h"
#include "core/text/sliced_glyph_atlas_assets.h"
#include "core/text/sliced_glyph_atlas_helpers.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"

namespace imp {
namespace sliced_glyph_atlas {
namespace {

using Entity = ::utils::Entity;
static constexpr std::array<uint16_t, 6> kBlitIndices = {0, 1, 2, 2, 3, 0};
static constexpr std::array<float2, 4> kBlitPositions = {
    float2{+1.f, +1.f},
    float2{-1.f, +1.f},
    float2{-1.f, -1.f},
    float2{+1.f, -1.f},
};
static constexpr std::array<float2, 4> kBlitTexCoords = {
    float2{1.f, 1.f},
    float2{0.f, 1.f},
    float2{0.f, 0.f},
    float2{1.f, 0.f},
};
}  // namespace

Future<std::unique_ptr<SlicedGlyphTextureManager>>
SlicedGlyphTextureManager::CreateAsync(BaseView& view, uint2 atlas_size,
                                       uint2 grid_size,
                                       Texture* composite_texture) {
  return view.GetAssetManager()
      .LoadMaterial(sliced_glyph_atlas_assets::kBlitSliceMaterialCmat)
      .Then(
          [&view, atlas_size, grid_size,
           composite_texture](AssetPtr<MaterialAsset> blit_material) {
            return std::unique_ptr<SlicedGlyphTextureManager>(
                new SlicedGlyphTextureManager(view, blit_material, atlas_size,
                                              grid_size, composite_texture));
          },
          Executor::Type::kForeground);
}

std::unique_ptr<Texture> SlicedGlyphTextureManager::CreateCompositeTexture(
    BaseView& view, uint2 atlas_size, uint2 grid_size) {
  filament::Engine& engine = *view.GetSharedEngine();
  uint2 composite_size = atlas_size * grid_size;
  using Format = TextureFactory::Format;
  using Usage = TextureFactory::Usage;

  // Create the composite texture that slices will blit into.
  std::unique_ptr<Texture> composite_texture =
      view.GetTextureFactory().CreateTexture(
          composite_size.x, composite_size.y, Format::RGBA8,
          Usage::DEFAULT | Usage::COLOR_ATTACHMENT | Usage::BLIT_SRC |
              Usage::BLIT_DST);

  // Create a fractlish checkerboard on checkerboard pattern to initialize the
  // composite texture.
  size_t composite_size_bytes =
      composite_size.x * composite_size.y * sizeof(uint32_t);
  void* checkerboard_buffer = malloc(composite_size_bytes);
  constexpr uint32_t kA = 0x40404040;
  constexpr uint32_t kB = 0x80808080;
  constexpr uint32_t kC = 0xC0C0C0C0;
  constexpr uint32_t kD = 0x00000000;
  for (size_t y = 0; y < composite_size.y; ++y) {
    for (size_t x = 0; x < composite_size.x; ++x) {
      uint32_t* pixel = reinterpret_cast<uint32_t*>(checkerboard_buffer) +
                        y * composite_size.x + x;
      *pixel = ((x / 8 + y / 8) % 2 ? ((x / 32 + y / 32) % 2 ? kA : kB)
                                    : ((x / 32 + y / 32) % 2 ? kC : kD)) +
               ((x + y) % 2 ? 0 : 0x10101010);
    }
  }
  // Make a simple local type to manage the lifetime of the checkerboard buffer
  // while we're waiting for the filament thread to service the setImage call.
  struct Packet {
    int refs;
  }* packet = new Packet{1};

  // Filament will call this callback after the setImage call has been completed
  // on the filament thread. At that point, we can release the checkerboard
  // buffer and the packet.
  auto cb = [](void* checkerboard_buffer, size_t size, void* user) {
    Packet* packet = reinterpret_cast<Packet*>(user);
    if (!--packet->refs) {
      free(checkerboard_buffer);
      delete packet;
    }
  };
  using PixelBufferDescriptor = filament::Texture::PixelBufferDescriptor;

  // Request the texture contents to be set.
  composite_texture->GetTexture()->setImage(
      engine, 0,
      PixelBufferDescriptor(checkerboard_buffer, composite_size_bytes,
                            PixelBufferDescriptor::PixelDataFormat::RGBA,
                            PixelBufferDescriptor::PixelDataType::UBYTE,
                            nullptr, cb, packet));

  return composite_texture;
}

SlicedGlyphTextureManager::SlicedGlyphTextureManager(
    BaseView& view, AssetPtr<MaterialAsset> blit_material, uint2 atlas_size,
    uint2 grid_size, Texture* composite_texture)
    : atlas_size_(atlas_size),
      grid_size_(grid_size),
      engine_(*view.GetSharedEngine()),
      composite_texture_(composite_texture) {
  auto& entity_manager = utils::EntityManager::get();
  uint2 composite_size = atlas_size * grid_size;

  blit_camera_ = engine_.createCamera(entity_manager.create());
  blit_camera_->setProjection(filament::Camera::Projection::ORTHO, 0.0,
                              atlas_size.x, atlas_size.y, 0.0, 0.0, 1.0);

  filament::LinearToneMapper linear_tone_mapper;
  linear_color_grading_ = filament::ColorGrading::Builder()
                              .toneMapper(&linear_tone_mapper)
                              .build(engine_);
  blit_render_target_ =
      filament::RenderTarget::Builder{}
          .texture(filament::RenderTarget::AttachmentPoint::COLOR,
                   composite_texture_->GetTexture())
          .build(engine_);

  blit_index_buffer_ = filament::IndexBuffer::Builder()
                           .indexCount(kBlitIndices.size())
                           .bufferType(filament::IndexBuffer::IndexType::USHORT)
                           .build(engine_);
  blit_index_buffer_->setBuffer(
      engine_,
      filament::IndexBuffer::BufferDescriptor(
          kBlitIndices.data(), kBlitIndices.size() * sizeof(kBlitIndices[0])));

  static_assert(kBlitPositions.size() == kBlitTexCoords.size());
  blit_vertex_buffer_ =
      filament::VertexBuffer::Builder()
          .vertexCount(kBlitPositions.size())
          .bufferCount(2)
          .attribute(filament::VertexAttribute::POSITION, 0,
                     filament::VertexBuffer::AttributeType::FLOAT2, 0)
          .attribute(filament::VertexAttribute::UV0, 1,
                     filament::VertexBuffer::AttributeType::FLOAT2, 0)
          .build(engine_);
  blit_vertex_buffer_->setBufferAt(
      engine_, 0,
      filament::VertexBuffer::BufferDescriptor(
          kBlitPositions.data(),
          kBlitPositions.size() * sizeof(kBlitPositions[0])));
  blit_vertex_buffer_->setBufferAt(
      engine_, 1,
      filament::VertexBuffer::BufferDescriptor(
          kBlitTexCoords.data(),
          kBlitTexCoords.size() * sizeof(kBlitTexCoords[0])));

  blit_material_ = blit_material;

  uint8_t slice_count = grid_size.x * grid_size.y;
  for (auto i = 0u; i < slice_count; ++i) {
    auto slice = blit_views_.Append(engine_.createView());
    auto* slice_view = blit_views_.back();
    float2 slice_offset, slice_scale;

    blit_scenes_.push_back(engine_.createScene());

    GetGridSliceOffsetAndScale(grid_size_, slice, &slice_offset, &slice_scale);
    int32_t left = slice_offset.x * composite_size.x;
    int32_t bottom = slice_offset.y * composite_size.y;
    uint32_t width = atlas_size.x;
    uint32_t height = atlas_size.y;

    slice_view->setScene(blit_scenes_.back());
    slice_view->setCamera(blit_camera_);
    slice_view->setRenderTarget(blit_render_target_);
    slice_view->setViewport(filament::Viewport(left, bottom, width, height));
    slice_view->setColorGrading(linear_color_grading_);
    slice_view->setDithering(filament::View::Dithering::NONE);
    slice_view->setPostProcessingEnabled(false);
    slice_view->setFrustumCullingEnabled(false);

    blit_materials_.push_back(
        view.GetMaterialFactory().CreateMaterial(blit_material_));

    blit_entities_.push_back(entity_manager.create());
    blit_scenes_.back()->addEntity(blit_entities_.back());

    filament::RenderableManager::Builder(1)
        .castShadows(false)
        .receiveShadows(false)
        .culling(false)
        .priority(1)
        .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES,
                  blit_vertex_buffer_, blit_index_buffer_)
        .material(0, blit_materials_.back()->GetFilamentMaterialInstance())
        .build(engine_, blit_entities_.back());
  }
}

SlicedGlyphTextureManager::~SlicedGlyphTextureManager() {
  auto& entity_manager = utils::EntityManager::get();
  for (Entity& entity : blit_entities_) {
    entity_manager.destroy(entity);
  }

  Entity camera_entity = blit_camera_->getEntity();
  engine_.destroyCameraComponent(camera_entity);
  entity_manager.destroy(camera_entity);

  blit_materials_.clear();
  engine_.destroy(blit_index_buffer_);
  engine_.destroy(blit_vertex_buffer_);
  engine_.destroy(linear_color_grading_);
  for (auto& scene : blit_scenes_) {
    engine_.destroy(scene);
  }
  engine_.destroy(blit_render_target_);
  for (filament::View* view : blit_views_) {
    engine_.destroy(view);
  }
  blit_material_.Reset();
}

void SlicedGlyphTextureManager::RenderSlice(filament::Renderer& renderer,
                                            SliceId slice) {
  renderer.render(blit_views_[slice]);
}

void SlicedGlyphTextureManager::PrepareBlit(SliceId slice,
                                            BorrowedTexturePtr texture) {
  if (texture) {
    blit_materials_[slice]->SetParameter(
        "Slice", texture,
        filament::TextureSampler(filament::TextureSampler::MagFilter::NEAREST));
  }
}

}  // namespace sliced_glyph_atlas
}  // namespace imp
