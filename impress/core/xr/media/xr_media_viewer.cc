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

#include "core/xr/media/xr_media_viewer.h"

#include <sys/stat.h>

#include <array>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/robin_map.h"
#include "core/config.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/media/media_type.h"
#include "core/ncsb/component_handle.h"
#include "core/render/android/android_defines.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/video/video_color_space.h"
#include "core/video/video_controller.h"
#include "core/video/video_controller_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"
#include "core/xr/media/data/assets.h"
#include "core/xr/media/utils.h"
#include "core/xr/media/xr_media_viewer_state.proto.imp.h"

namespace imp {

Future<absl::Status> XrMediaViewer::Setup() {
  if (state_.asset_url.empty()) {
    return Future<absl::Status>(
        absl::FailedPreconditionError("Media asset has not been specified."));
  }

  // Setup the renderer.
  if (state_.shape_type == XrMediaViewerState::ShapeType::USE_EXISTING) {
    if (!GetNode()->GetComponent<MeshRenderer>()) {
      return Future<absl::Status>(
          absl::FailedPreconditionError("XrMediaViewerState's shape_type is "
                                        "set to 'USE_EXISTING' but no "
                                        "renderer was found on the node."));
    }
    if (generated_node_) {
      generated_node_->SetEnabled(false);
    }
    mesh_renderer_ = GetNode()->GetComponent<MeshRenderer>();
  } else {
    if (!generated_node_) {
      generated_node_ = GetNode()->CreateChildNode();
    }
    generated_node_->SetEnabled(true);
    LoadRendererFromStateData();
    if (!mesh_renderer_) {
      return Future<absl::Status>(
          absl::InternalError("Failed to load renderer from state data."));
    }
  }

  // Setup the asset.
  switch (GetCurrentMediaType()) {
    case MediaType::kImage:
      return LoadMaterial().Then([this]() { return LoadImage(); });
      break;
    case MediaType::kVideo:
      return LoadMaterial().Then([this]() { return LoadVideo(); });
      break;
    case MediaType::kUnknown:
      return Future<absl::Status>(absl::FailedPreconditionError(
          "XrMediaViewer cannot load file of unknown type."));
      break;
  }
}

Future<absl::Status> XrMediaViewer::LoadImage() {
  return GetView()
      .GetAssetManager()
      .LoadImage(state_.asset_url)
      .Then([this](AssetPtr<imp::ImageAsset> source) {
        ScaleRenderer({source->GetWidth(), source->GetHeight()});
        SetTexture(GetView().GetTextureFactory().CreateTexture(*source));
      });
}

void XrMediaViewer::UpdateMediaStereoType() {
  // If the stereo type is set in the state, prioritize it over the media stereo
  // mode retrieved from the media itself. This allows the user to override the
  // media stereo mode if needed, for example to play a multiview video in
  // monoscopic mode.
  if (state_.stereo_type != XrMediaViewerState::StereoType::UNSET) return;

  MediaType media_type = GetCurrentMediaType();

  // If media type is unknown, default to monoscopic.
  // Image does not support stereo type detection. If the Image stereo type
  // is not set in the state, default to monoscopic.
  if (media_type != MediaType::kVideo) {
    state_.stereo_type = XrMediaViewerState::StereoType::MONOSCOPIC;
    return;
  }

  // Get the media stereo mode for video from the video controller. If failed,
  // default to monoscopic.
  MediaStereoMode stereo_mode = GetVideoController()->GetMediaStereoMode();

  // A simple static cast is not possible here because the enum values are not
  // the same.
  switch (stereo_mode) {
    case MediaStereoMode::kUnknown:
    case MediaStereoMode::kMonoscopic:
      state_.stereo_type = XrMediaViewerState::StereoType::MONOSCOPIC;
      break;
    case MediaStereoMode::kTopBottom:
      state_.stereo_type = XrMediaViewerState::StereoType::TOP_BOTTOM;
      break;
    case MediaStereoMode::kLeftRight:
      state_.stereo_type = XrMediaViewerState::StereoType::LEFT_RIGHT;
      break;
    case MediaStereoMode::kStereoMesh:
      state_.stereo_type = XrMediaViewerState::StereoType::STEREO_MESH;
      break;
    case MediaStereoMode::kInterleavedLeftPrimary:
      state_.stereo_type =
          XrMediaViewerState::StereoType::INTERLEAVED_LEFT_PRIMARY;
      break;
    case MediaStereoMode::kInterleavedRightPrimary:
      state_.stereo_type =
          XrMediaViewerState::StereoType::INTERLEAVED_RIGHT_PRIMARY;
      break;
    case MediaStereoMode::kInterleavedLeftPrimaryWithDepth:
      state_.stereo_type =
          XrMediaViewerState::StereoType::INTERLEAVED_LEFT_PRIMARY_WITH_DEPTH;
      break;
    case MediaStereoMode::kInterleavedRightPrimaryWithDepth:
      state_.stereo_type =
          XrMediaViewerState::StereoType::INTERLEAVED_RIGHT_PRIMARY_WITH_DEPTH;
      break;
  }
}

void XrMediaViewer::UpdateMaterialVideoColorSpaceParameters() {
  Material* material = mesh_renderer_->GetMaterial();
  material->TrySetParameter(kIsVideoParameter, true);

  // If the color space information is unknown, set it to default values.
  video::VideoColorSpace video_color_space = video_color_space_;
  if (video_color_space.GetStandard() ==
      video::VideoColorSpace::Standard::kUnknown) {
    video_color_space.SetStandard(video::VideoColorSpace::Standard::kBT709);
  }
  if (video_color_space.GetTransfer() ==
      video::VideoColorSpace::Transfer::kUnknown) {
    video_color_space.SetTransfer(video::VideoColorSpace::Transfer::kGamma_2_2);
  }
  if (video_color_space.GetRange() == video::VideoColorSpace::Range::kUnknown) {
    video_color_space.SetRange(video::VideoColorSpace::Range::kLimited);
  }

  // Set the color space parameters.
  if (absl::StatusOr<imp::mat3f> color_transform_matrix_sRGB =
          video_color_space.GetColorTransformMatrixSRGB();
      color_transform_matrix_sRGB.ok()) {
    material->TrySetParameter(kColorTransformMatrixSRGBParameter,
                              *color_transform_matrix_sRGB);
  } else {
    material->TrySetParameter(kColorTransformMatrixSRGBParameter,
                              kIdentityMat3f);
  }
  if (absl::StatusOr<imp::mat3f> color_transform_matrix_display_p3 =
          video_color_space.GetColorTransformMatrixDisplayP3();
      color_transform_matrix_display_p3.ok()) {
    material->TrySetParameter(kColorTransformMatrixDisplayP3Parameter,
                              *color_transform_matrix_display_p3);
  } else {
    material->TrySetParameter(kColorTransformMatrixDisplayP3Parameter,
                              kIdentityMat3f);
  }

  // Set the transfer function.
  material->TrySetParameter(kTransferFunctionParameter,
                            static_cast<int>(video_color_space.GetTransfer()));

  int maxContentLightLevel = video_color_space.GetMaxContentLightLevel();
  material->TrySetParameter(kMaxContentLightLevelParameter,
                            maxContentLightLevel);
}

MediaStereoMode XrMediaViewer::GetCurrentMediaStereoMode() const {
  switch (GetCurrentMediaType()) {
    case MediaType::kImage:
      // Image does not support stereo type detection.
      return MediaStereoMode::kUnknown;
    case MediaType::kVideo:
      // Get the media stereo mode for video from the video controller.
      return GetVideoController()->GetMediaStereoMode();
    case MediaType::kUnknown:
      return MediaStereoMode::kUnknown;
  }
}

Future<absl::Status> XrMediaViewer::LoadVideo() {
  Future<absl::Status> video_load_future;
  if (!GetVideoController().IsValid()) {
    // If there is no video controller, create it. Creating the video controller
    // also loads the video asset.
    VideoControllerState controller_state;
    controller_state.asset = state_.asset_url;
    controller_state.drm_license_url = state_.drm_license_url;
    controller_state.drm_scheme_uuid = state_.drm_scheme_uuid;

    switch (state_.stereo_type) {
      case XrMediaViewerState::StereoType::MONOSCOPIC:
      case XrMediaViewerState::StereoType::TOP_BOTTOM:
      case XrMediaViewerState::StereoType::LEFT_RIGHT:
      case XrMediaViewerState::StereoType::STEREO_MESH:
        controller_state.multiview_mode = VideoControllerState::MONOSCOPIC;
        break;
      case XrMediaViewerState::StereoType::INTERLEAVED_LEFT_PRIMARY:
      case XrMediaViewerState::StereoType::INTERLEAVED_RIGHT_PRIMARY:
        controller_state.multiview_mode = VideoControllerState::MULTIVIEW;
        break;
      default:
        controller_state.multiview_mode = VideoControllerState::UNSET;
        break;
    }

    video_load_future =
        GetRenderer()
            ->GetNode()
            ->AddComponentWithState<VideoController>(controller_state)
            .Then([&](ComponentHandle<VideoController> video_controller) {
              return absl::OkStatus();
            });
  } else {
    // Otherwise, reload the video asset using the existing video controller.
    video_load_future = GetVideoController()->LoadVideoAsset(
        state_.asset_url, state_.drm_license_url.value_or(""),
        state_.drm_scheme_uuid.value_or(""));
  }

  return video_load_future.Then([this]() {
    // Scale the renderer
    ScaleRenderer(GetVideoController()->GetVideoSize().value());

    // Set the stereo type and color space parameters.
    UpdateMediaStereoType();
    mesh_renderer_->GetMaterial()->TrySetParameter(
        kStereoTypeParameter, static_cast<int>(state_.stereo_type));
    if (GetCurrentMediaType() == MediaType::kVideo) {
      UpdateMaterialVideoColorSpaceParameters();
    }

    // Set the textures for the video source.
#if IMP_PLATFORM(ANDROID) && \
    defined(IMP_ANDROID_EXTERNAL_TEXTURE_SURFACE_USES_IMAGE_READER)
    switch (state_.stereo_type) {
      case XrMediaViewerState::StereoType::MONOSCOPIC:
      case XrMediaViewerState::StereoType::TOP_BOTTOM:
      case XrMediaViewerState::StereoType::LEFT_RIGHT:
      case XrMediaViewerState::StereoType::STEREO_MESH:
        SetTexture(GetVideoController()->GetVideoTexture());
        break;
      case XrMediaViewerState::StereoType::UNSET:
        SetTextures(GetVideoController()->GetVideoTextures());
        break;
      case XrMediaViewerState::StereoType::INTERLEAVED_LEFT_PRIMARY:
      case XrMediaViewerState::StereoType::INTERLEAVED_RIGHT_PRIMARY:
      case XrMediaViewerState::StereoType::INTERLEAVED_LEFT_PRIMARY_WITH_DEPTH:
      case XrMediaViewerState::StereoType::INTERLEAVED_RIGHT_PRIMARY_WITH_DEPTH:
        auto textures = GetVideoController()->GetVideoTextures();
        // If an auxiliary texture is explicitly requested but not available,
        // provide the primary texture instead. This allows custom materials
        // that expect two textures to work as intended.
        if (!textures.contains(SurfaceViewType::kAuxiliaryView)) {
          textures[SurfaceViewType::kAuxiliaryView] =
              textures[SurfaceViewType::kPrimaryView];
        }
        SetTextures(textures);
        break;
    }
#else
    SetTexture(GetVideoController()->GetVideoTexture());
#endif
  });
}

void XrMediaViewer::ScaleRenderer(int2 scale) {
  if (state_.shape_type == XrMediaViewerState::ShapeType::USE_EXISTING) {
    return;
  }
  if (mesh_renderer_ && scale.x > 0 && scale.y > 0) {
    float2 adjusted_scale = {scale.x, scale.y};
    if (state_.stereo_type == XrMediaViewerState::StereoType::LEFT_RIGHT) {
      adjusted_scale.x = adjusted_scale.x * .5f;
    } else if (state_.stereo_type ==
               XrMediaViewerState::StereoType::TOP_BOTTOM) {
      adjusted_scale.y = adjusted_scale.y * .5f;
    }
    float ratio = adjusted_scale.x / adjusted_scale.y;
    mesh_renderer_->GetNode()->SetLocalScale({ratio, 1, 1});
  }
}

MediaType XrMediaViewer::GetCurrentMediaType() const {
  if (HasImageExtension(state_.asset_url)) {
    return MediaType::kImage;
  } else if (HasVideoExtension(state_.asset_url)) {
    return MediaType::kVideo;
  }
  return MediaType::kUnknown;
}

Future<absl::Status> XrMediaViewer::ReloadViewer(XrMediaViewerState state) {
  state_ = state;

  // If the new asset is an image and the video controller still exists,
  // remove video controller.
  if (HasImageExtension(state.asset_url) && GetVideoController().IsValid()) {
    GetRenderer()->GetNode()->RemoveComponent<VideoController>();
  }

  return Setup();
}

void XrMediaViewer::SetTexture(imp::Texture* texture) {
  Material* material = mesh_renderer_->GetMaterial();
  material->TrySetParameter(kTextureParameter, texture);
  // If required by the material, fill the remaining material textures with
  // placeholder textures.
  for (absl::string_view param_name : std::array<const absl::string_view, 3>{
           kAuxiliaryTextureParameter, kPrimaryDepthTextureParameter,
           kSecondaryDepthTextureParameter}) {
    material->TrySetParameter(
        param_name, GetView().GetTextureFactory().BorrowPlaceholderTexture());
  }
}

void XrMediaViewer::SetTexture(TexturePtr texture) {
  mesh_renderer_->GetMaterial()->TrySetParameter(kTextureParameter,
                                                 std::move(texture));
}
void XrMediaViewer::SetTextures(
    RobinMap<SurfaceViewType, imp::Texture*> textures) {
  Material* material = mesh_renderer_->GetMaterial();
  material->TrySetParameter(kTextureParameter,
                            textures[SurfaceViewType::kPrimaryView]);

  // If required by the material, populate the remaining material textures with
  // the textures received from the video controller for the respective view
  // types, or with placeholder textures if the corresponding texture is not
  // available.
  const std::array<std::pair<absl::string_view, SurfaceViewType>, 3>
      kParamToViewType = {std::make_pair(kAuxiliaryTextureParameter,
                                         SurfaceViewType::kAuxiliaryView),
                          std::make_pair(kPrimaryDepthTextureParameter,
                                         SurfaceViewType::kPrimaryViewDepth),
                          std::make_pair(kSecondaryDepthTextureParameter,
                                         SurfaceViewType::kAuxiliaryViewDepth)};

  for (const auto& [param_name, view_type] : kParamToViewType) {
    if (textures.contains(view_type)) {
      material->TrySetParameter(param_name, textures[view_type]);
    } else {
      material->TrySetParameter(
          param_name, GetView().GetTextureFactory().BorrowPlaceholderTexture());
    }
  }
}

Future<absl::Status> XrMediaViewer::LoadMaterial() {
  // Either use a provided custom material or grab a stock material.
  absl::StatusOr<absl::string_view> material = GetMaterialAssetUrl();
  if (!material.ok()) {
    return Future<absl::Status>(material.status());
  }

  // Once the material has finished loading,
  // set it to the renderer and setup parameters.
  return GetView().GetMaterialFactory().LoadMaterial(*material).Then(
      [this](MaterialPtr material) {
        material->TrySetParameter(kStereoTypeParameter,
                                  static_cast<int>(state_.stereo_type));
        mesh_renderer_->SetMaterial(std::move(material));
      });
}

void XrMediaViewer::LoadRendererFromStateData() {
  // If we are using an existing mesh, then just return.
  if (!generated_node_ || !generated_node_->IsEnabled()) {
    return;
  }
  mesh_renderer_ = generated_node_->GetComponent<MeshRenderer>();

  // Determine if the shape_type is for spatial media or not.
  bool is_shape_type_spatial =
      state_.shape_type == XrMediaViewerState::ShapeType::FULL360 ||
      state_.shape_type == XrMediaViewerState::ShapeType::VR180;
  // Check the old value of our 'spatial' flag.
  bool is_old_shape_type_spatial = mesh_renderer_ && is_spatial_;
  // Update the value of our 'spatial' flag.
  is_spatial_ = is_shape_type_spatial;

  // If we have no renderer yet, or the current media has a different
  // spatial property from the last media, create a new renderer.
  if (!mesh_renderer_ || is_shape_type_spatial != is_old_shape_type_spatial) {
    MeshRenderer::FrustumCullingMode culling_mode =
        is_shape_type_spatial ? MeshRenderer::FrustumCullingMode::kDisabled
                              : MeshRenderer::FrustumCullingMode::kEnabled;
    mesh_renderer_ = generated_node_->AddComponent<MeshRenderer>(culling_mode);

    mesh_renderer_->SetMesh(GetView().GetMeshFactory().CreateQuad());

    if (is_shape_type_spatial) {
      mesh_renderer_->SetShadowCastingMode(MeshRenderer::ShadowMode::kNone);
      mesh_renderer_->SetShadowReceivingMode(MeshRenderer::ShadowMode::kNone);
      // We need to set the priority to 0 so that it renders behind everything.
      mesh_renderer_->SetPriority(0);
    }
  }
}

ComponentHandle<MeshRenderer> XrMediaViewer::GetRenderer() const {
  return mesh_renderer_;
}

ComponentHandle<VideoController> XrMediaViewer::GetVideoController() const {
  if (!GetRenderer().IsValid()) return ComponentHandle<VideoController>();
  return GetRenderer()->GetNode()->GetComponent<VideoController>();
}

absl::StatusOr<absl::string_view> XrMediaViewer::GetMaterialAssetUrl() {
  if (state_.custom_material.has_value()) {
    return state_.custom_material.value();
  }

  switch (state_.shape_type) {
    case XrMediaViewerState::ShapeType::FULL360:
      return xrmedia::kXrMediaSkybox360Cmat.GetUrl();
    case XrMediaViewerState::ShapeType::VR180:
      return xrmedia::kXrMediaSkybox180Cmat.GetUrl();
    case XrMediaViewerState::ShapeType::DEFAULT_FLAT:
    case XrMediaViewerState::ShapeType::USE_EXISTING:
      return xrmedia::kXrMediaPlaneCmat.GetUrl();
  }
  return absl::FailedPreconditionError(
      "XrMediaViewer GetMaterialAssetUrl did not find a material for the "
      "state's given shape type.");
}

void XrMediaViewer::Update(const FrameTime& frame_time) {
  if (GetCurrentMediaType() != MediaType::kVideo) return;

  video::VideoColorSpace surface_color_space =
      GetVideoController()->GetSourceColorSpace();
  if (video_color_space_ != surface_color_space) {
    IMP_LOG(imp::INFO) << "New video color space detected: Previous: "
              << video_color_space_.ToString()
              << "; New: " << surface_color_space.ToString();
    video_color_space_ = surface_color_space;
    UpdateMaterialVideoColorSpaceParameters();
  }
}

}  // namespace imp
