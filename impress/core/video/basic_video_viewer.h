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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_BASIC_VIDEO_VIEWER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_BASIC_VIDEO_VIEWER_H_

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/isf_info.h"
#include "core/video/basic_video_viewer_state.proto.imp.h"
#include "core/video/video_controller.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp {

// Creates a simple video display on a quad with a VideoController.
// Takes in a video asset and optionally a custom material.
class BasicVideoViewer : public Component {
 public:
  Future<absl::Status> SetupWithState();
  void Cleanup();

  // Access to the VideoController.
  ComponentHandle<VideoController> GetController() { return video_controller_; }
  // Access to the renderer's material; useful for setting parameters
  // if a custom material was passed in (ex: alpha).
  Material* GetMaterial() { return mesh_renderer_->GetMaterial(); }

 private:
  ComponentHandle<MeshRenderer> mesh_renderer_;
  ComponentHandle<VideoController> video_controller_;
  BasicVideoViewerState state_;

  void OnVideoLoaded();

 public:
  using IsfInfo = IsfInfo<&BasicVideoViewer::state_>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_BASIC_VIDEO_VIEWER_H_
