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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_CAMERA_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_CAMERA_RENDERER_H_

#include <vector>

#include "core/ar/ar_session.h"
#include "core/model/mesh/mesh_data.h"
#include "core/ncsb/component.h"

namespace imp {

// A Component that renders a background camera texture from a given ArSession.
//
// The camera texture is rendered as a background on the View so that all other
// rendered objects will be rendered on top.
//
// When rendering the camera texture, the virtual camera should be moved to
// match the tracked position of the physical camera as provided by the
// ArSession. Typically, the class ArSceneController is used to control both the
// creation of this component and the movement of the virtual camera.
//
// Note: The component is added asynchronously to support loading the material
// used to render the texture on the background thread.
//
// Example:
//   camera_renderer_node = CreateNode();
//   camera_renderer_node->AddComponent<imp::ArCameraRenderer>(ar_session);
class ArCameraRenderer : public Component {
 public:
  Future<absl::Status> Setup(const ar::ArSession* ar_session);

  void Update(const imp::FrameTime& frame_time);

  // Rebuilds the mesh used to render the camera texture using ArCamera UV API.
  // Must be called when the device orientation or screen resolution changes.
  // Typically, this is handled automatically by the ArSceneController.
  // Example:
  //   ar_session->SetDisplayGeometry(0, 1080, 1920, 0.01, 1000);
  //   // Must call Update() for the changes to take effect.
  //   ar_session->Update();
  //   ar_camera_renderer->RebuildMesh();
  void RebuildMesh();

 private:
  std::unique_ptr<MeshData> MakeMeshData() const;

  const ar::ArSession* ar_session_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_AR_AR_CAMERA_RENDERER_H_
