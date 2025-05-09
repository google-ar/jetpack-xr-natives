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

#include "core/view/ar/ar_camera_renderer.h"

#include <memory>
#include <utility>

#include "core/ar/ar_session.h"
#include "core/common/trace.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_description.h"
#include "core/view/ar/ar_assets.h"
#include "core/view/framework/render/material.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/framework/view.h"

namespace imp {

namespace {

using VertexAttribute = VertexFormat::VertexAttribute;
using AttributeType = VertexFormat::AttributeType;

// We render fullscreen using a triangle instead of a quad for performance.
// See:
// (broken link)/

// Triangle vertices in OpenGL NDC Coordinates. The rectangle from (-1, -1, 1)
// to (1, 1, 1) is visible. (-1, 1) is the top left. The orientation of the
// triangle is selected to match the orientation of android views.
static constexpr std::array<float3, 3> kNormalizedDeviceCoords = {
    float3{-1, 1, 1}, float3{-1, -3, 1}, float3{3, 1, 1}};

// The triangle vertices in normalized Android View coordinates. (0, 0) is top
// left of screen.
static constexpr std::array<float2, 3> kNormalizedViewCoords = {
    float2{0, 0}, float2{0, 2}, float2{2, 0}};

static constexpr std::array<uint16_t, 3> kIndices = {0, 1, 2};

static constexpr char kCameraTextureMaterialParameter[] = "cameraTexture";

}  // namespace

Future<absl::Status> ArCameraRenderer::Setup(const ar::ArSession* ar_session) {
  IMP_TRACE();
  ar_session_ = ar_session;

  ComponentHandle<ArCameraRenderer> self = GetHandle(this);
  return GetView()
      .GetAssetManager()
      .LoadMaterial(ar_assets::kCameraMaterialCmat)
      .Then([self](const imp::AssetPtr<imp::MaterialAsset>& mat_asset) mutable {
        IMP_TRACE_BLOCK("Then");
        // Create the material and assign the Camera texture to it.
        MaterialPtr material =
            self->GetView().GetMaterialFactory().CreateMaterial(mat_asset);
        material->SetParameter(kCameraTextureMaterialParameter,
                               self->ar_session_->camera_texture());

        // Create the mesh.
        MeshFactory& mesh_factory = self->GetView().GetMeshFactory();
        MeshPtr mesh = mesh_factory.CreateByMovingMeshData(
            filament::backend::PrimitiveType::TRIANGLES, self->MakeMeshData(),
            Box());

        // Create the MeshRenderer and assign the material and mesh to it.
        ComponentHandle<MeshRenderer> mesh_renderer =
            self->GetNode()->AddComponent<MeshRenderer>(
                MeshRenderer::FrustumCullingMode::kDisabled);
        mesh_renderer->SetMesh(std::move(mesh));
        mesh_renderer->SetMaterial(std::move(material));

        // Assign renderable properties.
        mesh_renderer->SetShadowCastingMode(MeshRenderer::ShadowMode::kNone);
        mesh_renderer->SetShadowReceivingMode(MeshRenderer::ShadowMode::kNone);
        mesh_renderer->SetPriority(0);

        return absl::OkStatus();
      });
}

void ArCameraRenderer::Update(const imp::FrameTime& frame_time) {
  IMP_TRACE();
  ComponentHandle<MeshRenderer> mesh_renderer =
      GetNode()->GetComponent<MeshRenderer>();
  if (!mesh_renderer) {
    return;
  }

  Material* material = mesh_renderer->GetMaterial();
  if (!material) {
    return;
  }

  // The texture used to render the camera can change each frame, so we set the
  // texture on the material each frame.
  material->SetParameter(kCameraTextureMaterialParameter,
                         ar_session_->camera_texture());
}

std::unique_ptr<MeshData> ArCameraRenderer::MakeMeshData() const {
  static_assert(kNormalizedDeviceCoords.size() == kNormalizedViewCoords.size());

  const VertexFormat kVertexFormat = {
      {VertexAttribute::POSITION, AttributeType::FLOAT3},
      {VertexAttribute::UV0, AttributeType::FLOAT2}};

  const MeshDescription kMeshDescription = {
      kVertexFormat, MeshDescription::IndexType::USHORT,
      kNormalizedDeviceCoords.size(), kIndices.size()};

  auto mesh_data = std::make_unique<MeshData>(kMeshDescription);

  std::vector<float2> device_oriented_texture_coords(
      kNormalizedViewCoords.size());
  std::vector<float2> texture_coords(kNormalizedViewCoords.size());

  // Convert from view normalized coords to device oriented normalized texture
  // coordinates. The range is (0, 0) to (1, 1). (0, 0) is the top left corner
  // of the image. These coordinates are rotated by the orientation of the
  // device.
  ar_session_->GetCameraTextureUVs(kNormalizedViewCoords,
                                   &device_oriented_texture_coords);

  for (size_t i = 0; i < mesh_data->GetDescription().vertex_count; ++i) {
    // Convert to OpenGL texture coordinates.
    // The range is (0,0) to (1,1). (0,1) is the top left of image.
    texture_coords[i] = float2(device_oriented_texture_coords[i].x,
                               1.0f - device_oriented_texture_coords[i].y);

    mesh_data->VertexAttributeAt<float3>(i, VertexAttribute::POSITION) =
        kNormalizedDeviceCoords[i];
    mesh_data->VertexAttributeAt<float2>(i, VertexAttribute::UV0) =
        texture_coords[i];
  }

  for (size_t i = 0; i < mesh_data->GetDescription().index_count; ++i) {
    mesh_data->IndexAt<uint16_t>(i) = kIndices[i];
  }

  return mesh_data;
}

void ArCameraRenderer::RebuildMesh() {
  // Technically, the only part of the mesh that can change is the uvs, but
  // we re-create the whole mesh data and move it to upload it to the gpu.
  // This way, we don't need to hold onto the mesh data memory to guarantee it
  // lives long enough to finish uploading to the gpu. Since this only happens
  // infrequently and the mesh is tiny, it's not a problem to re-create it.
  ComponentHandle<MeshRenderer> mesh_renderer =
      GetNode()->GetComponent<MeshRenderer>();
  mesh_renderer->GetMesh()->UpdateMeshData(MakeMeshData());
}

}  // namespace imp
