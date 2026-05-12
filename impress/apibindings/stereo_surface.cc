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

#include <algorithm>
#include <memory>
#include <tuple>
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
constexpr float2 kDefaultCornerRadius = kZero2;
// Render priority for the surface is set to 5 to ensure they render after the
// environment. The environment renders at default priority 4 and panels at 6.
constexpr int kRenderPriorityBetweenEnvironmentAndPanels = 5;
// The name of the node created to host the mesh collider as a workaround for
// older system images.
static constexpr absl::string_view kMeshColliderWorkaroundNodeName =
    "MeshColliderWorkaroundNode";
}  // namespace

absl::Status StereoSurface::Setup(MediaStereoMode stereo_mode,
                                  MediaBlendingMode blending_mode,
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
  blending_mode_ = blending_mode;
  use_super_sampling_ = use_super_sampling;

  mesh_renderer_left_or_both_ = GetNode()->AddComponent<MeshRenderer>(
      MeshRenderer::FrustumCullingMode::kDisabled);
  // Start Disabled until the material is set and we have a canvas shape.
  mesh_renderer_left_or_both_->SetEnabled(false);
  mesh_renderer_left_or_both_->SetShadowCastingMode(
      MeshRenderer::ShadowMode::kNone);
  mesh_renderer_left_or_both_->SetShadowReceivingMode(
      MeshRenderer::ShadowMode::kNone);
  // Ensure the mesh is rendered after the environment to avoid
  // clipping.
  mesh_renderer_left_or_both_->SetPriority(
      kRenderPriorityBetweenEnvironmentAndPanels);

  right_eye_node_ = GetNode()->CreateChildNode();
  mesh_renderer_right_ = right_eye_node_->AddComponent<MeshRenderer>(
      MeshRenderer::FrustumCullingMode::kDisabled);
  mesh_renderer_right_->SetEnabled(false);
  mesh_renderer_right_->SetShadowCastingMode(MeshRenderer::ShadowMode::kNone);
  mesh_renderer_right_->SetShadowReceivingMode(MeshRenderer::ShadowMode::kNone);
  mesh_renderer_right_->SetPriority(kRenderPriorityBetweenEnvironmentAndPanels);

  material_future_both_ =
      InitializeMaterial(material_both_, RenderEyeTarget::kBoth);

  material_future_left_ =
      InitializeMaterial(material_left_, RenderEyeTarget::kLeftOnly);
  material_future_right_ =
      InitializeMaterial(material_right_, RenderEyeTarget::kRightOnly);

  per_eye_material_future_ =
      material_future_left_.Combine(material_future_right_);
  return absl::OkStatus();
}

Future<std::unique_ptr<android_xr::JxrMediaMaterial>>
StereoSurface::CreateJxrMediaMaterial(RenderEyeTarget eye_target,
                                      bool use_super_sampling,
                                      MediaBlendingMode blending_mode) {
  return android_xr::JxrMediaMaterial::Create(
      GetView(), MediaShapeType::kDefaultFlat, use_super_sampling, eye_target,
      blending_mode);
}

Future<absl::Status> StereoSurface::InitializeMaterial(
    std::unique_ptr<android_xr::JxrMediaMaterial>& material,
    RenderEyeTarget eye_target) {
  return CreateJxrMediaMaterial(eye_target, use_super_sampling_, blending_mode_)
      .Then([this, &material, stereo_mode = stereo_mode_](
                std::unique_ptr<android_xr::JxrMediaMaterial> result) {
        material = std::move(result);

        // Fetch all textures associated with the surface.
        auto textures = surface_->BorrowTextures();

        auto iter = textures.find(SurfaceViewType::kPrimaryView);
        if (iter != textures.end()) {
          material->SetPrimaryTexture(iter->second);
        }
        if (stereo_mode == MediaStereoMode::kInterleavedLeftPrimary ||
            stereo_mode == MediaStereoMode::kInterleavedRightPrimary ||
            stereo_mode == MediaStereoMode::kInterleavedLeftPrimaryWithDepth ||
            stereo_mode == MediaStereoMode::kInterleavedRightPrimaryWithDepth) {
          // Check for an (MV_HEVC) auxiliary texture.
          iter = textures.find(SurfaceViewType::kAuxiliaryView);
          if (iter != textures.end()) {
            material->SetAuxiliaryTexture(iter->second);
          }
        }

        material->SetStereoType(stereo_mode);
        // By default, fallback to the best-effort color conversion mode. We
        // switch to the user-specified color conversion mode when the user
        // explicitly sets the color information of the content to be
        // rendered to the surface.
        material->SetContentColorMetadata(MediaColorSpace());
        material->SetFeatherRadius(kDefaultFeatherRadius);
      });
}

void StereoSurface::Cleanup() {
  // Cleanup assigned textures on material to reset DRM mode.
  auto placeholder_texture =
      GetView().GetTextureFactory().BorrowPlaceholderTexture();

  if (material_future_both_.Ready()) {
    material_both_->SetPrimaryTexture(placeholder_texture);
    material_both_->SetAuxiliaryTexture(placeholder_texture);
    material_both_->SetPrimaryAlphaMask(placeholder_texture);
    material_both_->SetAuxiliaryAlphaMask(placeholder_texture);
  } else {
    material_future_both_.Cancel();
  }

  if (per_eye_material_future_.Ready()) {
    material_left_->SetPrimaryTexture(placeholder_texture);
    material_left_->SetAuxiliaryTexture(placeholder_texture);
    material_left_->SetPrimaryAlphaMask(placeholder_texture);
    material_left_->SetAuxiliaryAlphaMask(placeholder_texture);
    material_right_->SetPrimaryTexture(placeholder_texture);
    material_right_->SetAuxiliaryTexture(placeholder_texture);
    material_right_->SetPrimaryAlphaMask(placeholder_texture);
    material_right_->SetAuxiliaryAlphaMask(placeholder_texture);
  } else {
    per_eye_material_future_.Cancel();
  }

  CleanupColliderType();

  // Ensure the render component is destroyed before material_.
  GetNode()->RemoveComponent<MeshRenderer>();

  if (right_eye_node_) {
    GetView().DestroyNode(right_eye_node_);
    right_eye_node_ = NodeHandle();
  }
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
  auto shape_mesh = std::get_if<CustomMesh>(&canvas_shape);
  if (shape_q != nullptr) {
    is_per_eye_ = false;
    // Don't set the z scale to 0.0f, as that breaks the collider.
    GetNode()->SetLocalScale({shape_q->width, shape_q->height, 1.0f});
    if (is_different_shape) {
      mesh_renderer_left_or_both_->SetMesh(
          GetView().GetMeshFactory().CreateQuad({.size = float2(1.0f, 1.0f)}));
    }
  } else if (shape_s != nullptr) {
    is_per_eye_ = false;
    GetNode()->SetLocalScale(
        {shape_s->radius, shape_s->radius, shape_s->radius});
    if (is_different_shape) {
      mesh_renderer_left_or_both_->SetMesh(
          GetView().GetMeshFactory().CreateSphere(
              {.radius = 1.0f,
               .resolution = 50,
               .flip_uv = true,
               .flip_face_direction = true},
              MeshFactory::MeshDataStorageMode::kDiscardMeshData));
    }
  } else if (shape_h != nullptr) {
    is_per_eye_ = false;
    GetNode()->SetLocalScale(
        {shape_h->radius, shape_h->radius, shape_h->radius});
    if (is_different_shape) {
      mesh_renderer_left_or_both_->SetMesh(
          GetView().GetMeshFactory().CreateXYHemisphere(
              {.radius = 1.0f, .resolution = 50},
              // When not using the workaround, kStoreMeshData is required
              // because the MeshCollider on the same node directly references
              // the MeshData. When using the workaround, kDiscardMeshData is
              // used because the MeshCollider is on a separate child node with
              // its own MeshRenderer and MeshData.
              use_mesh_collider_workaround_
                  ? MeshFactory::MeshDataStorageMode::kDiscardMeshData
                  : MeshFactory::MeshDataStorageMode::kStoreMeshData));
    }
  } else if (shape_mesh != nullptr) {
    is_per_eye_ = shape_mesh->right_positions.has_value() &&
                  !shape_mesh->right_positions->empty();

    imp::CreateCustomMeshSettings settings{
        .positions = shape_mesh->left_positions,
        .texcoords = shape_mesh->left_texcoords};

    if (shape_mesh->left_indices.has_value() &&
        !shape_mesh->left_indices->empty()) {
      settings.indices = shape_mesh->left_indices;
    }

    settings.draw_mode = shape_mesh->draw_mode;

    mesh_renderer_left_or_both_->SetMesh(
        GetView().GetMeshFactory().CreateCustomMesh(
            settings, MeshFactory::MeshDataStorageMode::kDiscardMeshData));

    if (is_per_eye_) {
      imp::CreateCustomMeshSettings right_settings{
          .positions = *shape_mesh->right_positions,
          .texcoords = *shape_mesh->right_texcoords};
      if (shape_mesh->right_indices.has_value() &&
          !shape_mesh->right_indices->empty()) {
        right_settings.indices = shape_mesh->right_indices;
      }
      right_settings.draw_mode = settings.draw_mode;
      mesh_renderer_right_->SetMesh(GetView().GetMeshFactory().CreateCustomMesh(
          right_settings, MeshFactory::MeshDataStorageMode::kDiscardMeshData));
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

  // Set the corner radius if the shape is a quad, or zero otherwise.
  float2 corner_radius = kDefaultCornerRadius;
  if (shape_q != nullptr && shape_q->corner_radius > 0.f) {
    // Convert the corner radius to UV space.
    corner_radius = {shape_q->corner_radius / shape_q->width,
                     shape_q->corner_radius / shape_q->height};
    float max_radius = std::max(corner_radius.x, corner_radius.y);
    // Ensure that maximum radius doesn't exceed 0.5, preserving aspect ratio.
    corner_radius *= std::min(max_radius, 0.5f) / max_radius;
  }

  // Once we have a canvas shape set and the material is ready, we can enable
  // the mesh renderer.
  if (is_per_eye_) {
    per_eye_material_future_
        .Then([this, corner_radius]() {
          if (!material_left_ || !material_right_) {
            IMP_LOG(imp::ERROR) << "Failed to create per-eye materials.";
            return;
          }
          if (mesh_renderer_left_or_both_) {
            mesh_renderer_left_or_both_->SetMaterial(
                material_left_->GetMaterial());
            mesh_renderer_left_or_both_->SetEnabled(true);
          }
          if (mesh_renderer_right_) {
            mesh_renderer_right_->SetMaterial(material_right_->GetMaterial());
            mesh_renderer_right_->SetEnabled(true);
          }
          material_left_->SetCornerRadius(corner_radius);
          material_right_->SetCornerRadius(corner_radius);
        })
        .KeptBy(this);
  } else {
    material_future_both_
        .Then([this, corner_radius]() {
          if (!material_both_) {
            IMP_LOG(imp::ERROR) << "Failed to create both-eyes material.";
            return;
          }
          if (mesh_renderer_left_or_both_) {
            mesh_renderer_left_or_both_->SetMaterial(
                material_both_->GetMaterial());
            mesh_renderer_left_or_both_->SetEnabled(true);
          }
          if (mesh_renderer_right_) {
            mesh_renderer_right_->SetEnabled(false);
          }
          material_both_->SetCornerRadius(corner_radius);
        })
        .KeptBy(this);
  }
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
  stereo_mode_ = stereo_mode;
  material_future_both_ = material_future_both_.Then(
      [this, stereo_mode]() { material_both_->SetStereoType(stereo_mode); });
  per_eye_material_future_ =
      per_eye_material_future_.Then([this, stereo_mode]() {
        material_left_->SetStereoType(stereo_mode);
        material_right_->SetStereoType(stereo_mode);
      });
}

void StereoSurface::SetPrimaryAlphaMask(OwnedOrBorrowedTexturePtr alpha_mask) {
  BorrowedTexturePtr borrowed_alpha_mask = alpha_mask.Borrow();
  material_future_both_ =
      material_future_both_.Then([this, borrowed_alpha_mask]() mutable {
        if (!borrowed_alpha_mask) {
          // TODO: replace with white/black texture from the
          // TextureFactory
          borrowed_alpha_mask =
              GetView().GetTextureFactory().BorrowPlaceholderTexture();
        }
        material_both_->SetPrimaryAlphaMask(std::move(borrowed_alpha_mask));
      });
  per_eye_material_future_ =
      per_eye_material_future_.Then([this, borrowed_alpha_mask]() mutable {
        if (!borrowed_alpha_mask) {
          // TODO: replace with white/black texture from the
          // TextureFactory
          borrowed_alpha_mask =
              GetView().GetTextureFactory().BorrowPlaceholderTexture();
        }
        material_left_->SetPrimaryAlphaMask(borrowed_alpha_mask);
        material_right_->SetPrimaryAlphaMask(borrowed_alpha_mask);
      });
}

void StereoSurface::SetAuxiliaryAlphaMask(
    OwnedOrBorrowedTexturePtr auxiliary_alpha_mask) {
  BorrowedTexturePtr borrowed_auxiliary_alpha_mask =
      auxiliary_alpha_mask.Borrow();
  material_future_both_ = material_future_both_.Then(
      [this, borrowed_auxiliary_alpha_mask]() mutable {
        if (!borrowed_auxiliary_alpha_mask) {
          // TODO: replace with a white or black texture from the
          // TextureFactory
          borrowed_auxiliary_alpha_mask =
              GetView().GetTextureFactory().BorrowPlaceholderTexture();
        }
        material_both_->SetAuxiliaryAlphaMask(borrowed_auxiliary_alpha_mask);
      });
  per_eye_material_future_ = per_eye_material_future_.Then(
      [this, borrowed_auxiliary_alpha_mask]() mutable {
        if (!borrowed_auxiliary_alpha_mask) {
          // TODO: replace with a white or black texture from the
          // TextureFactory
          borrowed_auxiliary_alpha_mask =
              GetView().GetTextureFactory().BorrowPlaceholderTexture();
        }
        material_left_->SetAuxiliaryAlphaMask(borrowed_auxiliary_alpha_mask);
        material_right_->SetAuxiliaryAlphaMask(borrowed_auxiliary_alpha_mask);
      });
}

void StereoSurface::SetContentColorMetadata(MediaColorSpace color_space) {
  material_future_both_ = material_future_both_.Then([this, color_space]() {
    material_both_->SetContentColorMetadata(color_space);
  });
  per_eye_material_future_ =
      per_eye_material_future_.Then([this, color_space]() {
        material_left_->SetContentColorMetadata(color_space);
        material_right_->SetContentColorMetadata(color_space);
      });
}

void StereoSurface::SetFeatherRadius(const float2& feather_radius) {
  material_future_both_ = material_future_both_.Then([this, feather_radius]() {
    material_both_->SetFeatherRadius(feather_radius);
  });
  per_eye_material_future_ =
      per_eye_material_future_.Then([this, feather_radius]() {
        material_left_->SetFeatherRadius(feather_radius);
        material_right_->SetFeatherRadius(feather_radius);
      });
}

void StereoSurface::SetBlendingMode(MediaBlendingMode blending_mode) {
  if (blending_mode_ == blending_mode) {
    return;
  }
  blending_mode_ = blending_mode;
  RecreateMaterials();
}

void StereoSurface::RecreateMaterials() {
  material_future_both_ = material_future_both_.Then(
      // Capture the current values of the use_super_sampling_ and
      // blending_mode_ variables, as they may change by the time the lambda is
      // called.
      [this, use_super_sampling = use_super_sampling_,
       blending_mode = blending_mode_]() {
        return CreateJxrMediaMaterial(RenderEyeTarget::kBoth,
                                      use_super_sampling, blending_mode)
            .Then(
                [this](std::unique_ptr<android_xr::JxrMediaMaterial> material) {
                  if (!material) {
                    IMP_LOG(imp::ERROR) << "Failed to recreate both-eyes material";
                    return;
                  }
                  if (material_both_) {
                    material_both_->ApplyParametersTo(*material);
                  }
                  // Note that we must first set the new material on the mesh
                  // renderer before destroying the old material to avoid error
                  // from borrowed material ptr.
                  if (mesh_renderer_left_or_both_ && !is_per_eye_) {
                    mesh_renderer_left_or_both_->SetMaterial(
                        material->GetMaterial());
                  }
                  material_both_ = std::move(material);
                });
      });

  per_eye_material_future_ = per_eye_material_future_.Then(
      // Capture the current values of the use_super_sampling_ and
      // blending_mode_ variables, as they may change by the time the lambda is
      // called.
      [this, use_super_sampling = use_super_sampling_,
       blending_mode = blending_mode_]() {
        auto left_future = CreateJxrMediaMaterial(
            RenderEyeTarget::kLeftOnly, use_super_sampling, blending_mode);
        auto right_future = CreateJxrMediaMaterial(
            RenderEyeTarget::kRightOnly, use_super_sampling, blending_mode);

        return left_future.Merge(std::move(right_future))
            .Then(
                [this](std::tuple<std::unique_ptr<android_xr::JxrMediaMaterial>,
                                  std::unique_ptr<android_xr::JxrMediaMaterial>>
                           materials) {
                  auto [material_left, material_right] = std::move(materials);
                  if (!material_left || !material_right) {
                    IMP_LOG(imp::ERROR) << "Failed to recreate per-eye materials";
                    return;
                  }
                  if (material_left_) {
                    material_left_->ApplyParametersTo(*material_left);
                  }
                  if (material_right_) {
                    material_right_->ApplyParametersTo(*material_right);
                  }
                  // Note that we must first set the new material on the mesh
                  // renderer before destroying the old material to avoid error
                  // from borrowed material ptr.
                  if (is_per_eye_) {
                    if (mesh_renderer_left_or_both_) {
                      mesh_renderer_left_or_both_->SetMaterial(
                          material_left->GetMaterial());
                    }
                    if (mesh_renderer_right_) {
                      mesh_renderer_right_->SetMaterial(
                          material_right->GetMaterial());
                    }
                  }
                  material_left_ = std::move(material_left);
                  material_right_ = std::move(material_right);
                });
      });
}

}  // namespace imp
