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

#include "core/sprite/sprite_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/component_system.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/sprite/sprite_renderer_assets.h"
#include "core/sprite/sprite_renderer_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/device.h"
#include "core/view/utils/frame_time.h"
#include "core/view/view_events.h"

namespace imp {
namespace {

using RenderSpace = SpriteRendererState::RenderSpace;

constexpr float2 kDefaultPivotPosition = kOne2 * 0.5f;
constexpr float4 kDefaultSpriteColor = kOne4;
constexpr float2 kDefaultDisplayedUVOffset = kZero2;
constexpr float2 kDefaultDisplayedUVScale = kOne2;
constexpr size_t kMaxSpriteRendererForBlendOrder = 32767;

constexpr RenderSpace kDefaultRenderSpace = RenderSpace::SCREEN;

float2 GetDisplayedUVOffset(const SpriteRendererState& state) {
  if (state.uv_offset.has_value()) {
    return *state.uv_offset;
  } else {
    return kDefaultDisplayedUVOffset;
  }
}

float2 GetDisplayedUVScale(const SpriteRendererState& state) {
  if (state.uv_scale.has_value()) {
    return *state.uv_scale;
  } else {
    return kDefaultDisplayedUVScale;
  }
}

AssetDefinition GetDefaultMaterial(bool has_outline, bool is_external) {
  return is_external
             ? (has_outline
                    ? sprite_renderer_assets::kSpriteExternalOutlineMaterialCmat
                    : sprite_renderer_assets::kSpriteExternalMaterialCmat)
             : (has_outline ? sprite_renderer_assets::kSpriteOutlineMaterialCmat
                            : sprite_renderer_assets::kSpriteMaterialCmat);
}

}  // namespace

SpriteRenderer::System::System(BaseView* view)
    : ComponentSystem<SpriteRenderer>(view) {}

BorrowedMeshPtr SpriteRenderer::System::GetQuad() {
  return quad_mesh_.Borrow();
}

void SpriteRenderer::System::BeforeFirstComponentAdded() {
  quad_mesh_ =
      GetView().GetMeshFactory().CreateQuad({.size = kOne2, .flip_uv = true});
}

void SpriteRenderer::System::AfterLastComponentRemoved() { quad_mesh_.Reset(); }

void SpriteRenderer::System::PostComponentsUpdated(
    const FrameTime& frame_time) {
  std::vector<SpriteRenderer*> sorted_sprites;
  sorted_sprites.reserve(GetView()
                             .GetComponentManager()
                             .GetComponentPoolById(kComponentId<SpriteRenderer>)
                             ->GetComponentCount());
  GetComponentManager().UpdateEach<SpriteRenderer>(
      [&sorted_sprites](SpriteRenderer* sprite) {
        sorted_sprites.push_back(sprite);
      });
  std::sort(sorted_sprites.begin(), sorted_sprites.end(),
            [](SpriteRenderer* a, SpriteRenderer* b) {
              return a->GetNode()->GetWorldPosition().z <
                     b->GetNode()->GetWorldPosition().z;
            });

  for (size_t i = 0; i < sorted_sprites.size(); ++i) {
    sorted_sprites[i]->mesh_renderer_->SetBlendOrder(
        fmin(i, kMaxSpriteRendererForBlendOrder),
        MeshRenderer::BlendOrderMode::kGlobal);
  }
}

Future<absl::Status> SpriteRenderer::Setup() {
  if (GetView()
          .GetComponentManager()
          .GetComponentPoolById(kComponentId<SpriteRenderer>)
          ->GetComponentCount() >= kMaxSpriteRendererForBlendOrder) {
    IMP_LOG(imp::ERROR) << "Number of SpriteRenderer Components will exceed "
               << kMaxSpriteRendererForBlendOrder
               << ", z-order fighting for blend order may occur";
  }

  if (!state_.image.has_value() || state_.image->empty()) {
    if (state_.material.has_value()) {
      return GetView()
          .GetMaterialFactory()
          .LoadMaterial(*state_.material)
          .Then([this](OwnedMaterialPtr material) {
            SetupInternal(std::move(material));
          });
    }

    return Future<absl::Status>(absl::FailedPreconditionError(
        "Unable to create SpriteRenderer with no image url."));
  }

  return Setup(*state_.image);
}

void SpriteRenderer::Setup(MaterialPtr material) {
  SetupInternal(std::move(material));
}

void SpriteRenderer::Setup(OwnedMaterialPtr material) {
  SetupInternal(std::move(material));
}

Future<absl::Status> SpriteRenderer::Setup(const AssetDefinition& image) {
  Future<AssetPtr<ImageAsset>> image_asset_future =
      GetView().GetAssetManager().LoadImage(image);

  return image_asset_future.Then(
      [this](AssetPtr<ImageAsset> image_asset) mutable {
        BaseView& view = GetView();
        OwnedTexturePtr texture =
            view.GetTextureFactory().CreateTexture(*image_asset);

        return Setup(std::move(texture));
      });
}

Future<absl::Status> SpriteRenderer::Setup(absl::string_view url) {
  Future<AssetPtr<ImageAsset>> image_asset_future =
      GetView().GetAssetManager().LoadImage(url);

  return image_asset_future.Then([this](AssetPtr<ImageAsset> image) mutable {
    BaseView& view = GetView();
    OwnedTexturePtr texture = view.GetTextureFactory().CreateTexture(*image);

    return Setup(std::move(texture));
  });
}

Future<absl::Status> SpriteRenderer::Setup(const Texture* texture) {
  if (!texture) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "Unable to create SpriteRenderer with no texture."));
  }

  if (!texture->IsValid()) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "Unable to create SpriteRenderer with an invalid Filament texture"));
  }

  if (!state_.texture_size.has_value() &&
      !state_.physical_texture_size.has_value()) {
    SetTextureSizePhysicalPixels(texture->GetSize());
  }

  return LoadMaterial(*texture).Then(
      [this, texture](OwnedMaterialPtr material) {
        material->SetParameter("BaseColor", texture);
        SetupInternal(std::move(material));
      });
}

Future<absl::Status> SpriteRenderer::Setup(TexturePtr texture) {
  if (!state_.texture_size.has_value() &&
      !state_.physical_texture_size.has_value()) {
    SetTextureSizePhysicalPixels(texture->GetSize());
  }

  return LoadMaterial(*texture).Then(
      [this, texture(std::move(texture))](OwnedMaterialPtr material) mutable {
        material->SetParameter("BaseColor", std::move(texture));
        SetupInternal(std::move(material));
      });
}

Future<absl::Status> SpriteRenderer::Setup(OwnedTexturePtr texture) {
  if (!texture) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "Unable to create SpriteRenderer with empty owned texture."));
  }

  if (!state_.texture_size.has_value() &&
      !state_.physical_texture_size.has_value()) {
    SetTextureSizePhysicalPixels(texture->GetSize());
  }

  return LoadMaterial(*texture).Then(
      [this, texture(std::move(texture))](OwnedMaterialPtr material) mutable {
        material->SetParameter("BaseColor", std::move(texture));
        SetupInternal(std::move(material));
      });
}

Future<absl::Status> SpriteRenderer::Setup(BorrowedTexturePtr texture) {
  if (!texture) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "Unable to create SpriteRenderer with empty borrowed texture."));
  }

  if (!state_.texture_size.has_value() &&
      !state_.physical_texture_size.has_value()) {
    SetTextureSizePhysicalPixels(texture->GetSize());
  }

  return LoadMaterial(*texture).Then(
      [this, texture](OwnedMaterialPtr material) {
        material->SetParameter("BaseColor", texture);
        SetupInternal(std::move(material));
      });
}

void SpriteRenderer::SetupInternal(OwnedMaterialPtr material) {
  float4 color = state_.color.value_or(kDefaultSpriteColor);
  material->SetParameter("color", color);
  if (state_.outline_color.has_value()) {
    material->SetParameter("outlineColor", *state_.outline_color);
  }
  material->SetParameter("clipSpaceTransform", kIdentityMat4f);
  material->SetParameter(
      "displayedUVBounds",
      float4(GetDisplayedUVOffset(state_), GetDisplayedUVScale(state_)));

  mesh_node_ = GetNode()->CreateChildNode();
  mesh_renderer_ = mesh_node_->AddComponent<MeshRenderer>(
      MeshRenderer::FrustumCullingMode::kDisabled);
  mesh_renderer_->SetMesh(GetView()
                              .GetComponentManager()
                              .GetComponentSystem<SpriteRenderer>()
                              .GetQuad());
  if (state_.priority.has_value()) {
    mesh_renderer_->SetPriority(*state_.priority);
  }
  mesh_renderer_->SetMaterial(std::move(material));

  // There are cases where the physical texture size is known but the device
  // screen ratio is not known yet, so we wait till it's available to set the
  // virtual texture size.
  GetView().GetDispatcher().Connect(
      [handle = GetHandle(this)](
          const ViewSizeChangedEvent& view_size_changed_event) mutable {
        if (handle->GetView().GetDevice().IsPhysicalPixelRatioAvailable() &&
            !handle->state_.texture_size.has_value() &&
            handle->state_.physical_texture_size.has_value()) {
          handle->SetTextureSizePhysicalPixels(
              *handle->state_.physical_texture_size);
        }
      },
      this);
}

Future<OwnedMaterialPtr> SpriteRenderer::LoadMaterial(
    const Texture& texture) const {
  if (state_.material.has_value()) {
    return GetView()
        .GetMaterialFactory()
        .LoadMaterial(*state_.material)
        .Then([](MaterialPtr material) {
          return OwnedMaterialPtr(std::move(material));
        });
  } else {
    return GetView()
        .GetAssetManager()
        .LoadMaterial(GetDefaultMaterial(state_.outline_color.has_value(),
                                         IsExternal(texture)))
        .Then([this](AssetPtr<MaterialAsset> material) -> OwnedMaterialPtr {
          return GetView().GetMaterialFactory().CreateMaterial(material);
        });
  }
}

void SpriteRenderer::Update(const FrameTime& frame_time) {
  UpdateClipSpaceTransform();
}

void SpriteRenderer::OnActiveStatusChanged(bool is_active) {
  if (is_active) {
    // Update the clip_t_model of the sprite when the node just becomes active.
    // This can ensure Sprite component is rendering correctly no matter when
    // the node gets enabled.
    UpdateClipSpaceTransform();
  }
}

void SpriteRenderer::Cleanup() { GetView().DestroyNode(mesh_node_); }

void SpriteRenderer::SetColor(float4 color) {
  state_.color = color;
  mesh_renderer_->GetMaterial()->SetParameter("color", color);
}

float4 SpriteRenderer::GetColor() const { return *state_.color; }

void SpriteRenderer::SetOutlineColor(float4 color) {
  state_.outline_color = color;
  
  mesh_renderer_->GetMaterial()->SetParameter("outlineColor", color);
}

std::optional<float4> SpriteRenderer::GetOutlineColor() const {
  if (!state_.outline_color.has_value()) {
    return std::nullopt;
  }
  return *state_.outline_color;
}

float2 SpriteRenderer::GetTextureSize() const {
  if (state_.texture_size.has_value()) {
    return *state_.texture_size;
  }

  if (GetView().GetDevice().IsPhysicalPixelRatioAvailable() &&
      state_.physical_texture_size.has_value()) {
    return GetView().GetDevice().PhysicalPixelsToPixels(
               *state_.physical_texture_size) *
           GetDisplayedUVScale(state_);
  }

  IMP_LOG(imp::ERROR) << "Texture size queried when SpriteRenderer is not ready. "
                "Returning empty value";
  return kZero2;
}

float2 SpriteRenderer::GetTextureSizePhysicalPixels() const {
  if (GetView().GetDevice().IsPhysicalPixelRatioAvailable() &&
      state_.texture_size.has_value()) {
    return GetView().GetDevice().PixelsToPhysicalPixels(
        *state_.texture_size / GetDisplayedUVScale(state_));
  }

  // The virtual texture size is the default property to read for the texture
  // size, but in case that's not available, we check if the physical texture
  // size is available.
  if (state_.physical_texture_size.has_value()) {
    return *state_.physical_texture_size;
  }

  IMP_LOG(imp::ERROR)
      << "Physical texture size queried when SpriteRenderer is not ready. "
         "Returning empty value";
  return kZero2;
}

void SpriteRenderer::SetTextureSize(float2 size) {
  state_.texture_size = size;
  state_.physical_texture_size.reset();
}

void SpriteRenderer::SetTextureSizePhysicalPixels(float2 physical_pixels) {
  if (!GetView().GetDevice().IsPhysicalPixelRatioAvailable()) {
    state_.physical_texture_size = physical_pixels;
    return;
  }

  state_.physical_texture_size.reset();
  state_.texture_size =
      GetView().GetDevice().PhysicalPixelsToPixels(physical_pixels) *
      GetDisplayedUVScale(state_);
}

float2 SpriteRenderer::GetUVOffset() const {
  return GetDisplayedUVOffset(state_);
}

void SpriteRenderer::SetUVOffset(float2 uv_offset) {
  state_.uv_offset = uv_offset;
  mesh_renderer_->GetMaterial()->SetParameter(
      "displayedUVBounds",
      float4(GetDisplayedUVOffset(state_), GetDisplayedUVScale(state_)));
}

float2 SpriteRenderer::GetUVScale() const {
  return GetDisplayedUVScale(state_);
}

void SpriteRenderer::SetUVScale(float2 uv_scale) {
  state_.uv_scale = uv_scale;
  mesh_renderer_->GetMaterial()->SetParameter(
      "displayedUVBounds",
      float4(GetDisplayedUVOffset(state_), GetDisplayedUVScale(state_)));
}

void SpriteRenderer::SetTexture(const Texture* texture) {
  if (!texture) {
    return;
  }
  mesh_renderer_->GetMaterial()->SetParameter("BaseColor", texture);
}

float2 SpriteRenderer::GetPivot() const {
  return state_.pivot_position.value_or(kDefaultPivotPosition);
}

void SpriteRenderer::SetPivot(float2 pivot) { state_.pivot_position = pivot; }

SpriteRendererState::RenderSpace SpriteRenderer::GetRenderSpace() const {
  return state_.render_space.value_or(kDefaultRenderSpace);
}

Rect SpriteRenderer::GetLocalBounds() const {
  float2 size = GetTextureSize() * GetNode()->GetWorldScale().xy;
  float2 pivot_point = GetPivot() * float2(-1, 1) - float2(-0.5f, 0.5f);
  return Rect{.center = size * pivot_point, .half_extent = size / 2.0f};
}

void SpriteRenderer::UpdateClipSpaceTransform() {
  float2 viewport_dimensions = GetView().GetSize();
  float3 texture_size = float3{GetTextureSize(), 1};

  RenderSpace render_space = GetRenderSpace();

  if (render_space == RenderSpace::WORLD_TO_SCREEN) {
    const mat4f viewport_trs = Transform<float>(
                                   /*in_translation=*/float3(0.0f, 0.0f, 0),
                                   /*in_rotation=*/kIdentityQuatf,
                                   /*in_scale=*/
                                   float3(2.0f / viewport_dimensions.x,
                                          2.0f / viewport_dimensions.y, 1))
                                   .AsMat4();

    const float3 pivot_point(float2(0.5f) - GetPivot(), 0.0f);

    const mat4f pivot_trs = Transform<float>(
                                /*in_translation=*/pivot_point * texture_size,
                                /*in_rotation=*/kIdentityQuatf,
                                /*in_scale=*/texture_size)
                                .AsMat4();

    filament::Camera* camera =
        GetView().GetCameraManager().GetCamera()->GetCamera();
    const mat4 clip_from_world =
        camera->getProjectionMatrix() * camera->getViewMatrix();

    float4 clip_point = clip_from_world * GetNode()->GetWorldPositionPrecise();
    float3 clip_space_translation;
    if (clip_point.w < 0.0f) {
      // If the node is behind the camera (w <= 0), the homogeneous divide by w
      // will incorrectly flip the geometry in front of the camera. To prevent
      // this, we force the point sufficiently far behind the camera plane
      // prior to perspective division, ensuring hardware clipping drops the
      // geometry correctly.
      clip_space_translation = float3(clip_point.xy, -1000.0f);
    } else {
      clip_space_translation = clip_point.xyz / clip_point.w;
    }

    const mat4f node_trs = Transform<float>(
                               /*in_translation=*/clip_space_translation,
                               /*in_rotation=*/kIdentityQuatf,
                               /*in_scale=*/GetNode()->GetWorldScale())
                               .AsMat4();

    const mat4f clip_space_trs = node_trs * viewport_trs * pivot_trs;

    mesh_renderer_->GetMaterial()->SetParameter("clipSpaceTransform",
                                                clip_space_trs);
  } else if (render_space == RenderSpace::SCREEN) {
    const mat4f clip_t_viewport =
        Transform<float>(
            /*in_translation=*/float3(-1.0f, -1.0f, 0),
            /*in_rotation=*/kIdentityQuatf,
            /*in_scale=*/float3(2.0f, 2.0f, 1))
            .AsMat4();

    const mat4f texture_t_model = Transform<float>(
                                      /*in_translation=*/float3(0.5f, 0.5f, 0),
                                      /*in_rotation=*/kIdentityQuatf,
                                      /*in_scale=*/float3(1.0f, -1.0f, 1))
                                      .AsMat4();

    float2 viewport_dimensions = GetView().GetSize();

    const mat4f viewport_t_screen =
        Transform<float>(
            /*in_translation=*/float3(0, 1, 0),
            /*in_rotation=*/kIdentityQuatf,
            /*in_scale=*/
            float3(1.0f / viewport_dimensions.x, -1.0f / viewport_dimensions.y,
                   1))
            .AsMat4();

    const mat4f clip_t_model = clip_t_viewport * viewport_t_screen *
                               GetTextureToScreenMatrix() * texture_t_model;

    mesh_renderer_->GetMaterial()->SetParameter("clipSpaceTransform",
                                                clip_t_model);
  }
}

// TODO This method shouldn't be public - remove it
mat4f SpriteRenderer::GetTextureToScreenMatrix() const {
  const float3 pivot_s_texture = float3{GetTextureSize(), 1};

  const float3 texture_p_pivot(GetPivot(), 0.0f);
  const mat4f pivot_t_texture =
      Transform<float>(
          /*in_translation=*/-texture_p_pivot * pivot_s_texture,
          /*in_rotation=*/kIdentityQuatf,
          /*in_scale=*/pivot_s_texture)
          .AsMat4();
  if (GetView().IsPreciseTranslationEnabled()) {
    mat4 screen_t_pivot = GetNode()->GetWorldTrsPrecise();
    screen_t_pivot[3].z = 0.0f;
    return mat4f(screen_t_pivot * pivot_t_texture);
  } else {
    mat4f screen_t_pivot = GetNode()->GetWorldTrs();
    screen_t_pivot[3].z = 0.0f;
    return screen_t_pivot * pivot_t_texture;
  }
}

bool SpriteRenderer::IsExternal(const Texture& texture) const {
  return texture.GetTexture()->getTarget() ==
         filament::Texture::Sampler::SAMPLER_EXTERNAL;
}

}  // namespace imp
