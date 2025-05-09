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

#ifndef THIRD_PARTY_IMPRESS_CORE_XR_MEDIA_XR_MEDIA_VIEWER_H_
#define THIRD_PARTY_IMPRESS_CORE_XR_MEDIA_XR_MEDIA_VIEWER_H_

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/robin_map.h"
#include "core/math/vec.h"
#include "core/media/media_type.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/render/android/android_defines.h"
#include "core/render/texture.h"
#include "core/video/video_color_space.h"
#include "core/video/video_controller.h"
#include "core/view/base_view.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"
#include "core/xr/media/xr_media_viewer_state.proto.imp.h"

namespace imp {

class XrMediaViewer : public Component {
 public:
  Future<absl::Status> Setup();

  // Regenerates the Xr Media Viewer based on new state data.
  Future<absl::Status> ReloadViewer(XrMediaViewerState state);

  ComponentHandle<MeshRenderer> GetRenderer() const;
  ComponentHandle<VideoController> GetVideoController() const;

  MediaType GetCurrentMediaType() const;
  MediaStereoMode GetCurrentMediaStereoMode() const;

  void Update(const FrameTime& frame_time);

 private:
  absl::StatusOr<absl::string_view> GetMaterialAssetUrl();
  Future<absl::Status> LoadMaterial();
  void LoadRendererFromStateData();

  void SetTexture(imp::Texture* texture);
  void SetTexture(TexturePtr texture);

  // Sets the textures for each view. Only used if multiview playback support is
  // required.
  void SetTextures(RobinMap<SurfaceViewType, imp::Texture*> textures);

  // Updates the media stereo type based on the state and the metadata extracted
  // from the media file.
  void UpdateMediaStereoType();

  // Updates the material parameters based on the color space of the media.
  void UpdateMaterialVideoColorSpaceParameters();

  Future<absl::Status> LoadVideo();
  Future<absl::Status> LoadImage();

  void ScaleRenderer(int2 scale);

  // If the renderer was generated and owned by this component,
  // it will store the child node here.
  NodeHandle generated_node_;
  ComponentHandle<MeshRenderer> mesh_renderer_;
  XrMediaViewerState state_;
  bool is_spatial_;

  video::VideoColorSpace video_color_space_;

 public:
  using IsfInfo = IsfInfo<&XrMediaViewer::state_>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_XR_MEDIA_XR_MEDIA_VIEWER_H_
