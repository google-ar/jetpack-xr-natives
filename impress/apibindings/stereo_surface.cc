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

#include "apibindings/stereo_surface.h"

#include <sys/types.h>

#include <memory>
#include <utility>
#include <variant>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/render/android/android_defines.h"
#include "core/render/android/android_external_texture_surface.h"
#include "core/render/content_security_level.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/collision/collider_state.proto.imp.h"
#include "core/view/framework/collision/mesh_collider.h"
#include "core/view/framework/collision/sphere_collider.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "split_engine/materials/jxr_media_material.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace {
constexpr float2 kDefaultFeatherRadius = kZero2;
// Render priority for the surface is set to 5 to ensure they render after the
// environment. The environment renders at default priority 4 and panels at 6.
constexpr int kRenderPriorityBetweenEnvironmentAndPanels = 5;
// The name of the node created to host the mesh collider as a workaround for
// older system images.
static constexpr absl::string_view kMeshColliderWorkaroundNodeName =
    "MeshColliderWorkaroundNode";
}  // namespace

absl::Status StereoSurface::Setup(MediaStereoMode stereo_mode,
                                  ContentSecurityLevel content_security_level,
                                  bool use_super_sampling) {
  absl::Span<const SurfaceViewType> view_types =
      kAndroidExternalTextureSurfaceConfigMono;
  if (stereo_mode == MediaStereoMode::kInterleavedLeftPrimary ||
      stereo_mode == MediaStereoMode::kInterleavedRightPrimary) {
    view_types = kAndroidExternalTextureSurfaceConfigStereo;
  }
  MP_ASSIGN_OR_RETURN(surface_,
                   AndroidExternalTextureSurface::Create(
                       GetView(), content_security_level, view_types));
  stereo_mode_ = stereo_mode;

  mesh_renderer_ = GetNode()->AddComponent<MeshRenderer>(
      MeshRenderer::FrustumCullingMode::kDisabled);
  // Start Disabled until the material is set and we have a canvas shape.
  mesh_renderer_->SetEnabled(false);
  mesh_renderer_->SetShadowCastingMode(MeshRenderer::ShadowMode::kNone);
  mesh_renderer_->SetShadowReceivingMode(MeshRenderer::ShadowMode::kNone);

  material_future_ =
      android_xr::JxrMediaMaterial::Create(
          GetView(), MediaShapeType::kDefaultFlat, use_super_sampling)
          .Then([this, stereo_mode](
                    std::unique_ptr<android_xr::JxrMediaMaterial> material) {
            material_ = std::move(material);

            // Fetch all textures associated with the surface.
            auto textures = surface_->BorrowTextures();

            auto iter = textures.find(SurfaceViewType::kPrimaryView);
            if (iter != textures.end()) {
              material_->SetPrimaryTexture(iter->second);
            }
            if (stereo_mode == MediaStereoMode::kInterleavedLeftPrimary ||
                stereo_mode == MediaStereoMode::kInterleavedRightPrimary ||
                stereo_mode ==
                    MediaStereoMode::kInterleavedLeftPrimaryWithDepth ||
                stereo_mode ==
                    MediaStereoMode::kInterleavedRightPrimaryWithDepth) {
              // Check for an (MV_HEVC) auxiliary texture.
              iter = textures.find(SurfaceViewType::kAuxiliaryView);
              if (iter != textures.end()) {
                material_->SetAuxiliaryTexture(iter->second);
              }
            }

            material_->SetStereoType(stereo_mode);
            // By default, fallback to the best-effort color conversion mode. We
            // switch to the user-specified color conversion mode when the user
            // explicitly sets the color information of the content to be
            // rendered to the surface.
            material_->SetContentColorMetadata(MediaColorSpace());
            material_->SetFeatherRadius(kDefaultFeatherRadius);
            mesh_renderer_->SetMaterial(material_->GetMaterial());
            // Ensure the mesh is rendered after the environment to avoid
            // clipping.
            mesh_renderer_->SetPriority(
                kRenderPriorityBetweenEnvironmentAndPanels);
          });
  return absl::OkStatus();
}

void StereoSurface::Cleanup() {
  // Cleanup assigned textures on material to reset DRM mode.
  if (material_future_.Ready()) {
    auto placeholder_texture =
        GetView().GetTextureFactory().BorrowPlaceholderTexture();
    material_->SetPrimaryTexture(placeholder_texture);
    material_->SetAuxiliaryTexture(placeholder_texture);
    material_->SetPrimaryAlphaMask(placeholder_texture);
    material_->SetAuxiliaryAlphaMask(placeholder_texture);
  } else {
    material_future_.Cancel();
  }

  CleanupColliderType();

  // Ensure the render component is destroyed before material_.
  GetNode()->RemoveComponent<MeshRenderer>();
}

absl::Status StereoSurface::SetCanvasShape(const CanvasShape& canvas_shape) {
  // We should only create a new mesh if the shape type is different, otherwise
  // we'll just update the scale to match the new request
  bool is_different_shape = (canvas_shape.index() != canvas_shape_.index());
  canvas_shape_ = canvas_shape;

  // std::visit does not offer significant readability or
  // performance benefits here.
  auto shape_q = std::get_if<Quad>(&canvas_shape);
  auto shape_s = std::get_if<Sphere>(&canvas_shape);
  auto shape_h = std::get_if<Hemisphere>(&canvas_shape);
  if (shape_q != nullptr) {
    // Don't set the z scale to 0.0f, as that breaks the collider.
    GetNode()->SetLocalScale({shape_q->width, shape_q->height, 1.0f});
    if (is_different_shape) {
      mesh_renderer_->SetMesh(
          GetView().GetMeshFactory().CreateQuad({.size = float2(1.0f, 1.0f)}));
    }
  } else if (shape_s != nullptr) {
    GetNode()->SetLocalScale(
        {shape_s->radius, shape_s->radius, shape_s->radius});
    if (is_different_shape) {
      mesh_renderer_->SetMesh(GetView().GetMeshFactory().CreateSphere(
          {.radius = 1.0f,
           .resolution = 50,
           .flip_uv = true,
           .flip_face_direction = true},
          MeshFactory::MeshDataStorageMode::kDiscardMeshData));
    }
  } else if (shape_h != nullptr) {
    GetNode()->SetLocalScale(
        {shape_h->radius, shape_h->radius, shape_h->radius});
    if (is_different_shape) {
      mesh_renderer_->SetMesh(GetView().GetMeshFactory().CreateXYHemisphere(
          {.radius = 1.0f, .resolution = 50},
          // When not using the workaround, kStoreMeshData is required because
          // the MeshCollider on the same node directly references the MeshData.
          // When using the workaround, kDiscardMeshData is used because the
          // MeshCollider is on a separate child node with its own MeshRenderer
          // and MeshData.
          use_mesh_collider_workaround_
              ? MeshFactory::MeshDataStorageMode::kDiscardMeshData
              : MeshFactory::MeshDataStorageMode::kStoreMeshData));
    }
  } else {
    // In practice this should be impossible, since the higher level JXR APIs
    // don't have a value for this; Shape is a required field whenever setting
    // the state.

    CleanupColliderType();

    // early return to avoid avoid spuriously enabling the mesh.
    return absl::InvalidArgumentError(
        "monostate CanvasShape is not supported.");
  }

  // If the collider was enabled, get the collider type based on the canvas
  // shape and update the collider type.
  if (GetColliderEnabled()) {
    auto collider_type = GetColliderTypeByShape(canvas_shape);
    MP_RETURN_IF_ERROR(UpdateColliderType(collider_type));
  }

  // Once we have a canvas shape set and the matieral is ready, we can enable
  // the mesh renderer.
  material_future_ =
      material_future_.Then([this]() { mesh_renderer_->SetEnabled(true); });
  return absl::OkStatus();
}

StereoSurface::ColliderType StereoSurface::GetColliderTypeByShape(
    const CanvasShape& canvas_shape) {
  auto shape_q = std::get_if<Quad>(&canvas_shape);
  if (shape_q != nullptr) {
    return ColliderType::kPanel;
  }
  auto shape_s = std::get_if<Sphere>(&canvas_shape);
  if (shape_s != nullptr) {
    return ColliderType::kSphere;
  }
  auto shape_h = std::get_if<Hemisphere>(&canvas_shape);
  if (shape_h != nullptr) {
    return ColliderType::kMesh;
  }
  IMP_LOG(imp::ERROR) << "Collider for monostate CanvasShape is not supported.";
  return ColliderType::kUnknown;
}

absl::Status StereoSurface::UpdateColliderType(ColliderType collider_type) {
  if (collider_type_ == collider_type) {
    return absl::OkStatus();
  }

  // Remove the old collider
  CleanupColliderType();

  // Add the new collider
  switch (collider_type) {
    case ColliderType::kPanel:
      GetNode()->AddComponent<BoxCollider>()->SetBox({{}, {0.5f, 0.5f, 0.0f}});
      break;
    case ColliderType::kSphere:
      GetNode()->AddComponent<SphereCollider>()->SetSphere({{}, 1.0f});
      break;
    case ColliderType::kMesh: {
      auto collider_node = GetNode();
      if (use_mesh_collider_workaround_) {
        // Create a new node to host the mesh collider. Because MeshCollider
        // on old sys-image versions does not support rendering mesh format
        // created from the MeshFactory.
        collider_node = GetNode()->CreateChildNode();
        collider_node->SetName(kMeshColliderWorkaroundNodeName);
        // MeshRenderer here is only used to hold the mesh data for
        // MeshCollider, but not to be rendered.
        auto collider_node_mesh_renderer =
            collider_node->AddComponent<MeshRenderer>();
        collider_node_mesh_renderer->SetEnabled(false);
        collider_node_mesh_renderer->SetMesh(
            GetView().GetMeshFactory().CreateXYHemisphere(
                {.radius = 1.0f, .resolution = 50, .is_position_only = true},
                MeshFactory::MeshDataStorageMode::kStoreMeshData));
      }
      MP_RETURN_IF_ERROR(
          collider_node
              ->AddComponent<MeshCollider>(MeshColliderState::ColliderMode::
                                               COLLIDE_WITH_MESH_ONLY_DEFAULT)
              .status());
      break;
    }
    case ColliderType::kNone:
      // Attempting to clean up the collider. Do nothing.
      break;
    case ColliderType::kUnknown:
    default:
      IMP_LOG(imp::WARNING) << "Attempting to add an unknown collider type: "
                   << static_cast<int>(collider_type);
      break;
  }

  collider_type_ = collider_type;
  return absl::OkStatus();
}

void StereoSurface::CleanupColliderType() {
  switch (collider_type_) {
    case ColliderType::kPanel:
      GetNode()->RemoveComponent<BoxCollider>();
      break;
    case ColliderType::kSphere:
      GetNode()->RemoveComponent<SphereCollider>();
      break;
    case ColliderType::kMesh:
      if (use_mesh_collider_workaround_) {
        for (NodeHandle child : GetNode()->GetChildren()) {
          if (child->GetName() == kMeshColliderWorkaroundNodeName) {
            GetView().DestroyNode(child);
          }
        }
      } else {
        GetNode()->RemoveComponent<MeshCollider>();
      }
      break;
    case ColliderType::kNone:
      // Already cleaned up. Do nothing.
      break;
    case ColliderType::kUnknown:
    default:
      IMP_LOG(imp::WARNING) << "Attempting to clean up an unknown collider type: "
                   << static_cast<int>(collider_type_);
      break;
  }
  collider_type_ = ColliderType::kNone;
}

absl::Status StereoSurface::SetColliderEnabled(bool enable_collider) {
  if (!enable_collider) {
    CleanupColliderType();
    return absl::OkStatus();
  }

  ColliderType collider_type = GetColliderTypeByShape(canvas_shape_);
  MP_RETURN_IF_ERROR(UpdateColliderType(collider_type));
  return absl::OkStatus();
}

absl::StatusOr<android::Surface*> StereoSurface::GetSurface() {
  if (surface_ == nullptr) {
    return absl::InternalError("Surface is not initialized.");
  }
  return surface_->GetSurface();
}

absl::Status StereoSurface::SetSurfaceDimensions(int width, int height) {
  if (surface_ == nullptr) {
    return absl::InternalError("Surface is not initialized.");
  }
  return surface_->SetDefaultBufferSize({width, height});
}

void StereoSurface::SetStereoMode(MediaStereoMode stereo_mode) {
  material_future_ = material_future_.Then([this, stereo_mode]() {
    stereo_mode_ = stereo_mode;
    material_->SetStereoType(stereo_mode);
  });
}

void StereoSurface::SetPrimaryAlphaMask(OwnedOrBorrowedTexturePtr alpha_mask) {
  material_future_ = material_future_.Then(
      [this, alpha_mask = std::move(alpha_mask)]() mutable {
        if (!alpha_mask) {
          // TODO: replace with white/black texture from the
          // TextureFactory
          alpha_mask = GetView().GetTextureFactory().BorrowPlaceholderTexture();
        }
        material_->SetPrimaryAlphaMask(std::move(alpha_mask));
      });
}

void StereoSurface::SetAuxiliaryAlphaMask(
    OwnedOrBorrowedTexturePtr auxiliary_alpha_mask) {
  material_future_ = material_future_.Then(
      [this, auxiliary_alpha_mask = std::move(auxiliary_alpha_mask)]() mutable {
        if (!auxiliary_alpha_mask) {
          // TODO: replace with a white or black texture from the
          // TextureFactory
          auxiliary_alpha_mask =
              GetView().GetTextureFactory().BorrowPlaceholderTexture();
        }
        material_->SetAuxiliaryAlphaMask(std::move(auxiliary_alpha_mask));
      });
}

void StereoSurface::SetContentColorMetadata(MediaColorSpace color_space) {
  material_future_ = material_future_.Then([this, color_space]() {
    material_->SetContentColorMetadata(color_space);
  });
}

void StereoSurface::SetFeatherRadius(const float2& feather_radius) {
  material_future_ = material_future_.Then([this, feather_radius]() {
    material_->SetFeatherRadius(feather_radius);
  });
}

}  // namespace imp
