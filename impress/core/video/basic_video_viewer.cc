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

#include "core/video/basic_video_viewer.h"

#include <string>
#include <tuple>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/media/media_source.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/video/data/video_assets.h"
#include "core/video/video_controller.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp {

using State = ::imp::media::MediaSource::State;

constexpr absl::string_view kVideoTextureMaterialParameter = "videoTexture";

Future<absl::Status> BasicVideoViewer::SetupWithState() {
  if (state_.asset.empty()) {
    return Future<absl::Status>(
        absl::FailedPreconditionError("Video asset has not been specified."));
  }

  // Start loading the material.
  std::string material_url = state_.custom_material.value_or(
      std::string(video::kVideoMaterialCmat.GetUrl()));

  // Load the material and video controller.
  std::string drm_license_url =
      !state_.drm_license_url.has_value() ? "" : state_.drm_license_url.value();
  std::string drm_scheme_uuid =
      !state_.drm_scheme_uuid.has_value() ? "" : state_.drm_scheme_uuid.value();
  return GetView()
      .GetMaterialFactory()
      .LoadMaterial(material_url)
      .Merge(GetNode()->AddComponent<VideoController>(
          state_.asset, drm_license_url, drm_scheme_uuid))
      .Then([this](std::tuple<MaterialPtr, ComponentHandle<VideoController>>
                       result) {
        MaterialPtr& material = std::get<0>(result);
        video_controller_ = std::get<1>(result);

        // Setup the renderer.
        mesh_renderer_ = GetNode()->AddComponent<MeshRenderer>();
        // Create a quad with UV origin at top-left vertex to be consistent
        // with glb model mesh UV. Video material has flipUV as false and
        // thus will use the original UV.
        mesh_renderer_->SetMesh(
            GetView().GetMeshFactory().CreateQuad({.flip_uv = true}));
        mesh_renderer_->SetMaterial(std::move(material));

        OnVideoLoaded();

        // Add an event listener for when the video has been loaded.
        this->Connect([this](const VideoController::VideoLoadedEvent& event) {
          state_.asset = event.video_controller->GetAssetUrl();
          OnVideoLoaded();
        });
      });
}

void BasicVideoViewer::Cleanup() {
  GetNode()->RemoveComponent<MeshRenderer>();
  GetNode()->RemoveComponent<VideoController>();
}

void BasicVideoViewer::OnVideoLoaded() {
  const std::string material_param(kVideoTextureMaterialParameter);
  mesh_renderer_->GetMaterial()->SetParameter(
      material_param, GetController()->GetVideoTexture());
  auto size = GetController()->GetVideoSize();
  if (size->x > 0 && size->y > 0) {
    float ratio = static_cast<float>(size->x) / static_cast<float>(size->y);
    GetNode()->SetLocalScale({ratio, 1, 1});
  }
}

}  // namespace imp
