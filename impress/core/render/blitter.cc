/*
 * Copyright 2026 Google LLC
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

#include "core/render/blitter.h"

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/status_macros.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/filament/include/filament/View.h"
#include "filament/filament/include/filament/Viewport.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/utils/include/utils/EntityManager.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/render/blit_assets.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/window/filament_host.h"

namespace imp {

namespace {

// Filament view name for the blitter.
constexpr absl::string_view kBlitterViewName = "BlitterView";

// Uniform name for the source texture in the blit material.
constexpr absl::string_view kSourceTextureUniformName = "sourceTexture";

constexpr int kVertexCountForQuad = 4;
constexpr int kIndexCountForQuad = 6;

// Indices for the quad.
static constexpr std::array<uint16_t, kIndexCountForQuad> kBlitIndices = {
    0, 1, 2, 2, 3, 0};

// Vertex positions for the quad.
static constexpr std::array<float3, kVertexCountForQuad> kBlitPositions = {
    float3{-1.f, -1.f, 0.f},
    float3{+1.f, -1.f, 0.f},
    float3{+1.f, +1.f, 0.f},
    float3{-1.f, +1.f, 0.f},
};

// UV coordinates for the quad.
static constexpr std::array<float2, kVertexCountForQuad> kBlitTexCoords = {
    float2{0.f, 1.f},
    float2{1.f, 1.f},
    float2{1.f, 0.f},
    float2{0.f, 0.f},
};

}  // namespace

Future<std::unique_ptr<Blitter>> Blitter::Create(BaseView& view) {
  return view.GetAssetManager()
      .LoadMaterial(blit_assets::kBlitMaterialCmat)
      .Then([&view](absl::StatusOr<AssetPtr<MaterialAsset>> asset)
                -> absl::StatusOr<std::unique_ptr<Blitter>> {
        MP_RETURN_IF_ERROR(asset.status());

        OwnedMaterialPtr blit_material =
            view.GetMaterialFactory().CreateMaterial(*asset);

        return std::unique_ptr<Blitter>(
            new Blitter(view, std::move(blit_material)));
      });
}

Blitter::Blitter(BaseView& view, OwnedMaterialPtr blit_material)
    : view_(view), blit_material_(std::move(blit_material)) {
  filament::Engine& engine = *view_.GetSharedEngine();
  utils::EntityManager& em = utils::EntityManager::get();

  CreateVertexBuffer(engine);
  CreateIndexBuffer(engine);

  camera_entity_ = em.create();
  camera_ = engine.createCamera(camera_entity_);

  blit_view_ = view_.CreateFilamentView();
  blit_view_->setName(kBlitterViewName.data());
  blit_view_->setCamera(camera_);
  blit_view_->setPostProcessingEnabled(false);
  blit_view_->setDithering(filament::View::Dithering::NONE);

  blit_scene_ = engine.createScene();
  blit_quad_entity_ = em.create();
  blit_scene_->addEntity(blit_quad_entity_);
  blit_view_->setScene(blit_scene_);
}

Blitter::~Blitter() {
  filament::Engine& engine = *view_.GetSharedEngine();
  utils::EntityManager& em = utils::EntityManager::get();

  if (vb_) engine.destroy(vb_);
  if (ib_) engine.destroy(ib_);

  if (camera_) engine.destroyCameraComponent(camera_entity_);
  em.destroy(camera_entity_);

  if (blit_quad_entity_) {
    blit_scene_->remove(blit_quad_entity_);
    engine.destroy(blit_quad_entity_);
    em.destroy(blit_quad_entity_);
  }

  if (blit_scene_) engine.destroy(blit_scene_);
  if (blit_view_) engine.destroy(blit_view_);
}

void Blitter::Blit(BorrowedMaterialPtr material,
                   filament::RenderTarget* render_target,
                   std::optional<filament::Viewport> viewport) {
  filament::Engine& engine = *view_.GetSharedEngine();

  // Update material on the quad.
  filament::RenderableManager& rm = engine.getRenderableManager();
  if (!rm.hasComponent(blit_quad_entity_)) {
    filament::RenderableManager::Builder(1)
        .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, vb_,
                  ib_)
        .material(0, material->GetFilamentMaterialInstance())
        .castShadows(false)
        .receiveShadows(false)
        .culling(false)
        .build(engine, blit_quad_entity_);
  } else {
    filament::RenderableManager::Instance instance =
        rm.getInstance(blit_quad_entity_);
    rm.setMaterialInstanceAt(instance, 0,
                             material->GetFilamentMaterialInstance());
  }

  blit_view_->setRenderTarget(render_target);
  if (viewport) {
    blit_view_->setViewport(*viewport);
  }

  if (filament::Renderer* renderer = view_.GetHost()->GetRenderer()) {
    renderer->render(blit_view_);
  }
}

absl::Status Blitter::Blit(filament::RenderTarget* src,
                           filament::RenderTarget* dest,
                           std::optional<filament::Viewport> dest_viewport) {
  if (!src) {
    return absl::InvalidArgumentError(
        "Source RenderTarget cannot be null. Blitting from the swap chain is "
        "not supported.");
  }

  const filament::Texture* src_color_tex =
      src->getTexture(filament::RenderTarget::AttachmentPoint::COLOR);

  if (!src_color_tex) {
    return absl::InvalidArgumentError(
        "Source RenderTarget must have a color texture attachment.");
  }

  // Leaving this here for context rather than moving it to the top namespace.
  static const filament::TextureSampler sampler(
      // Nearest is being used here for two reasons:
      // 1. Avoids the cost of box filtering.
      // 2. RTs have the same resolution so we get a pixel-perfect copy.
      // Revisit if the resolutions of the RTs differ due to future changes.
      filament::TextureSampler::MinFilter::NEAREST,
      filament::TextureSampler::MagFilter::NEAREST,
      filament::TextureSampler::WrapMode::CLAMP_TO_EDGE);

  blit_material_->GetFilamentMaterialInstance()->setParameter(
      kSourceTextureUniformName.data(), src_color_tex, sampler);

  filament::Viewport vp;
  if (dest_viewport) {  // Use the provided viewport if it exists.
    vp = *dest_viewport;
  } else if (dest) {  // Otherwise, use the size of the destination texture.
    const filament::Texture* dest_color_tex =
        dest->getTexture(filament::RenderTarget::AttachmentPoint::COLOR);

    if (!dest_color_tex) {
      return absl::InvalidArgumentError(
          "Destination RenderTarget must have a color texture attachment.");
    }

    const int width = dest_color_tex->getWidth();
    const int height = dest_color_tex->getHeight();

    vp = filament::Viewport(0, 0, width, height);
  } else {  // Otherwise, use the size of the view.
    const float2 view_size = view_.GetSize();
    vp = filament::Viewport(0, 0, view_size.x, view_size.y);
  }

  Blit(blit_material_.Borrow(), dest, vp);
  return absl::OkStatus();
}

void Blitter::CreateVertexBuffer(filament::Engine& engine) {
  if (vb_) return;

  constexpr int kBufferCountForQuad = 2;  // Only Position and UV needed.
  constexpr int kPositionAttributeIndex = 0;
  constexpr int kUvAttributeIndex = 1;

  vb_ = filament::VertexBuffer::Builder()
            .vertexCount(kVertexCountForQuad)
            .bufferCount(kBufferCountForQuad)
            .attribute(filament::VertexAttribute::POSITION,
                       kPositionAttributeIndex,
                       filament::VertexBuffer::AttributeType::FLOAT3)
            .attribute(filament::VertexAttribute::UV0, kUvAttributeIndex,
                       filament::VertexBuffer::AttributeType::FLOAT2)
            .build(engine);

  vb_->setBufferAt(engine, kPositionAttributeIndex,
                   filament::VertexBuffer::BufferDescriptor(
                       kBlitPositions.data(), sizeof(kBlitPositions)));

  vb_->setBufferAt(engine, kUvAttributeIndex,
                   filament::VertexBuffer::BufferDescriptor(
                       kBlitTexCoords.data(), sizeof(kBlitTexCoords)));
}

void Blitter::CreateIndexBuffer(filament::Engine& engine) {
  if (ib_) return;

  ib_ = filament::IndexBuffer::Builder()
            .indexCount(kIndexCountForQuad)
            .bufferType(filament::IndexBuffer::IndexType::USHORT)
            .build(engine);

  ib_->setBuffer(engine, filament::IndexBuffer::BufferDescriptor(
                             kBlitIndices.data(), sizeof(kBlitIndices)));
}

}  // namespace imp
