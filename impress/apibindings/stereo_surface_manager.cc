// Copyright 2025 Google LLC
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

#include "apibindings/stereo_surface_manager.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "apibindings/bindings_texture.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/stereo_surface.h"
#include "core/common/small_source_location.h"
#include "core/input/pointer_event_processor.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace {

absl::StatusOr<ComponentHandle<StereoSurface>> GetStereoSurface(
    int32_t node_id) {
  NodeHandle node(utils::Entity::import(node_id));
  if (!node.IsValid()) {
    return absl::InvalidArgumentError("Node is not valid.");
  }
  auto result = node->GetComponent<StereoSurface>();
  if (!result.IsValid()) {
    return absl::InvalidArgumentError("Node is not a StereoSurface.");
  }
  return result;
}

}  // namespace

class StereoSurfaceManagerImpl : public StereoSurfaceManager {
 public:
  explicit StereoSurfaceManagerImpl(ImpressApiView& view);
  ~StereoSurfaceManagerImpl() override = default;

  absl::StatusOr<int32_t> CreateStereoSurfaceEntity(
      MediaStereoMode stereo_mode, MediaBlendingMode blending_mode,
      ContentSecurityLevel content_security_level,
      bool use_super_sampling) override;
  absl::Status SetStereoSurfaceEntityCanvasShape(
      int32_t node_id, StereoSurface::CanvasShape canvas_shape) override;
  absl::Status SetStereoSurfaceEntityColliderEnabled(
      int32_t node_id, bool enable_collider) override;
  absl::StatusOr<android::Surface*> GetSurfaceFromStereoSurfaceEntity(
      int32_t node_id) override;
  absl::Status SetSurfaceDimensionsForStereoSurfaceEntity(
      int32_t node_id, int32_t width, int32_t height) override;
  absl::Status SetFeatherRadiusForStereoSurfaceEntity(
      int32_t node_id, const float2& feather_radius) override;
  absl::Status SetStereoModeForStereoSurfaceEntity(
      int32_t node_id, MediaStereoMode stereo_mode) override;
  absl::Status SetBlendingModeForStereoSurfaceEntity(
      int32_t node_id, MediaBlendingMode blending_mode) override;
  absl::Status SetPrimaryAlphaMaskForStereoSurfaceEntity(
      int32_t node_id, int64_t alpha_mask_token) override;
  absl::Status SetAuxiliaryAlphaMaskForStereoSurfaceEntity(
      int32_t node_id, int64_t alpha_mask_token) override;
  absl::Status SetContentColorMetadataForStereoSurfaceEntity(
      int32_t node_id, MediaColorSpace color_space = {}) override;
  absl::Status SetSubViewConfigForStereoSurfaceEntity(int32_t node_id,
                                                      float bottom, float left,
                                                      float right,
                                                      float top) override;

 private:
  ImpressApiView& view_;
};

StereoSurfaceManagerImpl::StereoSurfaceManagerImpl(ImpressApiView& view)
    : view_(view) {}

absl::StatusOr<int32_t> StereoSurfaceManagerImpl::CreateStereoSurfaceEntity(
    MediaStereoMode stereo_mode, MediaBlendingMode blending_mode,
    ContentSecurityLevel content_security_level, bool use_super_sampling) {
  NodeHandle node = view_.CreateNode();
  absl::StatusOr<ComponentHandle<StereoSurface>> status =
      node->AddComponent<StereoSurface>(stereo_mode, blending_mode,
                                        content_security_level,
                                        use_super_sampling);
  if (!status.ok()) {
    return status.status();
  }
  if (!status->IsValid()) {
    return absl::InternalError("Node is not valid.");
  }
  return node.GetEntity().getId();
}

absl::Status StereoSurfaceManagerImpl::SetStereoSurfaceEntityCanvasShape(
    int32_t node_id, StereoSurface::CanvasShape canvas_shape) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  return stereo_surface->SetCanvasShape(canvas_shape);
}

absl::Status StereoSurfaceManagerImpl::SetStereoSurfaceEntityColliderEnabled(
    int32_t node_id, bool enable_collider) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  return stereo_surface->SetColliderEnabled(enable_collider);
}

absl::StatusOr<android::Surface*>
StereoSurfaceManagerImpl::GetSurfaceFromStereoSurfaceEntity(int32_t node_id) {
  absl::StatusOr<ComponentHandle<StereoSurface>> result =
      GetStereoSurface(node_id);
  if (!result.ok()) {
    return result.status();
  }
  return (*result)->GetSurface();
}

absl::Status
StereoSurfaceManagerImpl::SetSurfaceDimensionsForStereoSurfaceEntity(
    int32_t node_id, int32_t width, int32_t height) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  return stereo_surface->SetSurfaceDimensions(width, height);
}

absl::Status StereoSurfaceManagerImpl::SetFeatherRadiusForStereoSurfaceEntity(
    int32_t node_id, const float2& feather_radius) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetFeatherRadius(feather_radius);
  return absl::OkStatus();
}

absl::Status StereoSurfaceManagerImpl::SetStereoModeForStereoSurfaceEntity(
    int32_t node_id, MediaStereoMode stereo_mode) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetStereoMode(stereo_mode);
  return absl::OkStatus();
}

absl::Status StereoSurfaceManagerImpl::SetBlendingModeForStereoSurfaceEntity(
    int32_t node_id, MediaBlendingMode blending_mode) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetBlendingMode(blending_mode);
  return absl::OkStatus();
}

absl::Status
StereoSurfaceManagerImpl::SetPrimaryAlphaMaskForStereoSurfaceEntity(
    int32_t node_id, int64_t alpha_mask_token) {
  OwnedOrBorrowedTexturePtr alpha_mask;
  // If the alpha mask token is kUnSetAlphaMaskToken, then the alpha mask is
  // removed.
  if (alpha_mask_token != ImpressApiView::kUnSetAlphaMaskToken) {
    alpha_mask = view_.FromJava<BindingsTexture>(alpha_mask_token)
                     ->GetTexture(SmallSourceLocation::Current());
  }
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetPrimaryAlphaMask(std::move(alpha_mask));
  return absl::OkStatus();
}

absl::Status
StereoSurfaceManagerImpl::SetAuxiliaryAlphaMaskForStereoSurfaceEntity(
    int32_t node_id, int64_t alpha_mask_token) {
  OwnedOrBorrowedTexturePtr alpha_mask;
  // If the alpha mask token is kUnSetAlphaMaskToken, then the alpha mask is
  // removed.
  if (alpha_mask_token != ImpressApiView::kUnSetAlphaMaskToken) {
    alpha_mask = view_.FromJava<BindingsTexture>(alpha_mask_token)
                     ->GetTexture(SmallSourceLocation::Current());
  }
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetAuxiliaryAlphaMask(std::move(alpha_mask));
  return absl::OkStatus();
}

absl::Status
StereoSurfaceManagerImpl::SetContentColorMetadataForStereoSurfaceEntity(
    int32_t node_id, MediaColorSpace color_space) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  stereo_surface->SetContentColorMetadata(color_space);
  return absl::OkStatus();
}

absl::Status StereoSurfaceManagerImpl::SetSubViewConfigForStereoSurfaceEntity(
    int32_t node_id, float bottom, float left, float right, float top) {
  MP_ASSIGN_OR_RETURN(ComponentHandle<StereoSurface> stereo_surface,
                   GetStereoSurface(node_id));
  float offset_x = left;
  float offset_y = std::min(top, bottom);
  float width = right - left;
  float height = std::abs(top - bottom);
  stereo_surface->SetSubViewRects({offset_x, offset_y, width, height},
                                  {offset_x, offset_y, width, height});
  return absl::OkStatus();
}

std::unique_ptr<StereoSurfaceManager> CreateStereoSurfaceManager(
    ImpressApiView& view) {
  return std::make_unique<StereoSurfaceManagerImpl>(view);
}

}  // namespace imp
