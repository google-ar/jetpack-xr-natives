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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_MESH_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_MESH_RENDERER_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/model/mesh/mesh.h"
#include "core/ncsb/component.h"
#include "core/render/base_renderable_manager.h"

namespace imp {

// Renders a set of 3D meshes and materials in the scene at the position,
// rotation, and scale of the node this component is attached to.
//
// You can use a MeshRenderer to dynamically create a renderable. E.g.:
// view->GetAssetManager()
//     .LoadMaterial(kMyMaterial)
//     .Then([view](const imp::MaterialAsset* material) {
//       NodeHandle node = view->CreateNode();
//       auto mesh_renderer = node->AddComponent<imp::MeshRenderer>();
//       mesh_renderer->SetMesh(view->GetMeshFactory().CreateQuad());
//       mesh_renderer->SetMaterial(material->CreateMaterial());
//     }).KeptBy(view);
class MeshRenderer : public Component {
 public:
  // Determines if this MeshRenderer is culled when the bounding box of the
  // mesh is outside of the camera's view frustum.
  enum class FrustumCullingMode {
    // Default value. Culling is enabled.
    kEnabled,
    // Culling based on the view frustum of the camera is disabled.
    kDisabled
  };

  // Represents different modes that shadows that can be rendered with.
  // Used to represent how shadows are rendered on this object (shadow
  // receiving) as well as how this object influences the shadows rendered on
  // other objects (shadow casting).
  enum class ShadowMode {
    // Shadows with hard edges.
    kHardShadows,
    // No Shadows.
    kNone
  };

  // Used to determine how BlendOrder is interpreted. If local, it only applies
  // within a single MeshRenderer. If global, applies across all components.
  // Applied on a per-primitive basis.
  enum class BlendOrderMode {
    // Blend order applies to primitives within this component.
    kLocal,
    // Blend order applies to all components.
    kGlobal
  };

  // Determines how instance transforms are handled.
  enum class InstancingMode {
    // Use explicit instance transforms supplied on the CPU side. For this mode,
    // the max number of instances is limited to 64. The instance transforms
    // will be initialized to the identity matrix, and can be updated with
    // `UpdateInstanceTransformsInRange`.
    // Note: in this mode, the AABB of the renderable will be set to the union
    // of all instance AABBs.
    kCpuTransforms,
    // Use instance indices to compute instance transforms on the GPU. The max
    // number of instances is 32767 but explicit instance transforms cannot be
    // provided on the CPU side, and must be handled in the material's vertex
    // shader. For example, the material can set `instanced : true` and use
    // Filament's `getInstanceIndex()` in order to procedurally compute an
    // instance position, or use it to perform a texture sample to inform the
    // instance position, etc.
    // Note: in this mode, the AABB is computed as if there was only one
    // instance with an identity transform. You may need to disable frustum
    // culling in SetupOptions or on the material if you see incorrect culling
    // behavior, but this will have a performance impact.
    kGpuIndices,
  };

  MeshRenderer() {}

  void Cleanup();

  struct SetupOptions {
    size_t primitive_count = 1;
    FrustumCullingMode culling_mode = FrustumCullingMode::kEnabled;
    // The number of instances of the renderable. The max number of instances is
    // limited by Filament, see `InstancingMode`.
    size_t num_instances = 1;
    // The mode to use for instancing. Only used if `num_instances` is greater
    // than 1. This mode greatly affects the max number of instances that can
    // be used.
    InstancingMode instancing_mode = InstancingMode::kGpuIndices;
    // num_bones is the number of skinning bones for the entire renderable. It
    // is shared across primitives, and the bones will be initialized to the
    // identity matrix.
    uint8_t num_bones = 0;
  };

  // Creates a RenderableManager::Instance with |primitive_count| primitives and
  // the FrustumCulling mode set to kEnabled.
  void Setup(size_t primitive_count = 1);

  // Creates a RenderableManager::Instance with |primitive_count| primitives and
  // the FrustumCulling mode set to |culling_mode|.
  // TODO: Remove and replace with a SetCulling method after
  // filament exposes setCulling in the class filament::RenderableManager.
  void Setup(FrustumCullingMode culling_mode, size_t primitive_count = 1);

  // Creates a RenderableManager::Instance with:
  // If |SetupOptions.num_instances| is 1, then instanced rendering is disabled.
  // If |SetupOptions.num_bones| is zero, then skinning is disabled.
  // If |SetupOptions.num_bones| is non-zero, the mesh must have bone weights
  // and indices baked into its geometry as vertex attributes.
  void Setup(const SetupOptions& options);

  // Returns the number of instances as specified in SetupOptions.
  //
  // See SetupOptions for more details for how instancing can be used.
  size_t GetInstanceCount() const;

  // Sets the instance transforms for the specified MeshRenderer. Allows
  // updating a subset of the instance transforms in the range:
  // [first_instance_index, first_instance_index +
  // new_instance_transforms.size()].
  // Note: instancing _with instance transforms_ must be enabled at
  // `mesh_render` build time, with the `SetupOptions::num_instances` being
  // greater than 1 and `SetupOptions::instancing_mode` is `kCpuTransforms`.
  // Transforms are expected to be in the local space of the node.
  // Returns:
  // - UnavailableError if the MeshRenderer was not created with instancing or
  //   if the instancing mode is not kCpuTransforms.
  // - OutOfRangeError if the requested update is too large, or ends past the
  //   end of the instance transforms array.
  // - OkStatus otherwise.
  absl::Status UpdateInstanceTransformsInRange(
      absl::Span<const imp::mat4f> new_instance_transforms,
      size_t first_instance_index = 0);

  uint8_t GetBoneCount() const;

  // Sets the bone transforms for the specified MeshRenderer. Allows updating a
  // subset of the bones in the range:
  // - [first_bone_index, first_bone_index + new_bones.size()].
  // Note: skinning must be enabled at `mesh_render` build time, with the
  // `SetupOptions::num_bones`. Transforms are expected to be in the local space
  // of the node.
  // Returns:
  // - UnavailableError if the MeshRenderer was not created with bones.
  // - OutOfRangeError if the requested update is too large, or ends past the
  //   end of the bones array.
  // - OkStatus otherwise.
  absl::Status UpdateBoneTransformsInRange(
      absl::Span<const imp::mat4f> new_bones, uint8_t first_bone_index = 0);

  void OnActiveStatusChanged(bool is_active);

  size_t GetPrimitiveCount() const;

  // Sets the Material used to render the specified primitive.
  //
  // The MeshRenderer does not take ownership over the material. The material
  // must live until either the MeshRenderer is destroyed or SetMaterial is
  // called again.
  ABSL_DEPRECATED(
      "Use imp::BorrowedMaterialPtr overload instead. See "
      "(broken link).")
  void SetMaterial(Material* material, size_t primitive_index = 0);

  // Sets the Material used to render the specified primitive.
  //
  // The MeshRenderer takes ownership over the material.
  void SetMaterial(OwnedMaterialPtr material, size_t primitive_index = 0);

  // Sets the Material used to render the specified primitive.
  //
  // The MeshRenderer does not take ownership over the material. The material
  // must live until either the MeshRenderer is destroyed or SetMaterial is
  // called again.
  void SetMaterial(BorrowedMaterialPtr material, size_t primitive_index = 0);

  // Gets the material for the specified primitive if previously set, otherwise
  // returns nullptr.
  Material* GetMaterial(size_t primitive_index = 0) const;

  // Borrows the material for the specified primitive if previously set,
  // otherwise returns an empty material.
  //
  // Note: This will return an empty mesh if the mesh was assigned as MeshPtr or
  // Mesh*.
  BorrowedMaterialPtr BorrowMaterial(
      size_t primitive_index = 0,
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Sets the mesh for the specified primitive.
  //
  // The MeshRenderer does not take ownership over the mesh. The mesh
  // must live until either the MeshRenderer is destroyed or SetMesh is
  // called again.
  ABSL_DEPRECATED(
      "Use imp::BorrowedMeshPtr overload instead. See "
      "(broken link).")
  void SetMesh(Mesh* mesh, size_t primitive_index = 0);

  // Sets the Mesh for the specified primitive.
  //
  // The MeshRenderer takes ownership over the mesh.
  void SetMesh(OwnedMeshPtr mesh, size_t primitive_index = 0);

  // Sets the mesh for the specified primitive.
  //
  // The MeshRenderer does not take ownership over the mesh. The mesh
  // must live until either the MeshRenderer is destroyed or SetMesh is
  // called again.
  void SetMesh(BorrowedMeshPtr mesh, size_t primitive_index = 0);

  // Gets the mesh for the specified primitive if previously set, otherwise
  // returns nullptr.
  Mesh* GetMesh(size_t primitive_index = 0) const;

  // Borrows the mesh for the specified primitive if previously set,
  // otherwise returns an empty mesh.
  //
  // Note: This will return an empty mesh if the mesh was assigned as MeshPtr or
  // Mesh*.
  BorrowedMeshPtr BorrowMesh(
      size_t primitive_index = 0,
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Updates the AABB for this MeshRenderer with the union of all AABBs for
  // the meshes.
  void UpdateRenderableAabb();

  // Gets the AABB for this MeshRenderer.
  const Box& GetRenderableAabb() const;

  // Sets the shadow mode that this node can cast onto other nodes.
  // Shadow casting and shadow receiving do not impact each other. For instance,
  // this can cast shadows without receiving shadows.
  void SetShadowCastingMode(ShadowMode shadow_mode);

  // Gets the shadow mode that this node can cast onto other nodes.
  // The default value is ShadowMode::kNone.
  ShadowMode GetShadowCastingMode() const;

  // Sets the shadow mode that other nodes can cast onto this node.
  // Shadow casting and shadow receiving do not impact each other. For instance,
  // this can receive shadows without casting shadows.
  // If shadow receiving is kNone, this will not receive shadows no matter what
  // the shadow casting setting of other objects is set to.
  void SetShadowReceivingMode(ShadowMode shadow_mode);

  // Gets the shadow mode that other nodes can cast onto this node.
  // The default value is ShadowMode::kHardShadows.
  ShadowMode GetShadowReceivingMode() const;

  // Sets the priority, used to provide a limited amount of control over the
  // draw order of this node. Note that draw order is also impacted by blend
  // mode (i.e. transparent vs. opaque), channel, and culling.
  //
  // The priority is clamped to the range [0..7]. 0 is rendered first, 7 is
  // rendered last.
  void SetPriority(uint8_t priority);

  // Gets the priority that impacts the draw order of this node. See SetPriority
  // for more details.
  // The default value is 4.
  uint8_t GetPriority() const;

  // Sets the channel, used to provide a limited amount of control over the
  // draw order of this node. This takes precedence over setting the
  // Priority. Note that draw order is also impacted by blend mode (i.e.
  // transparent vs. opaque), priority, and culling.
  //
  // The channel is clamped to the range [0..3]. 0 is rendered first, 3 is
  // rendered last.
  void SetChannel(uint8_t channel);

  // Gets the channel that impacts the draw order of this node. See SetChannel
  // for more details.
  // The default value is 2.
  uint8_t GetChannel() const;

  // Enables/Disables the fog.
  void SetFogEnabled(bool enable);

  // Sets the blend order, used to provide a limited amount of control over the
  // draw order of this node. Only works in transparent blend mode.
  //
  // BlendOrderMode determines if the order is interpreted locally, relative to
  // this component only, or globally across all components.
  //
  // The order is clamped to the range [0..32767]. 0 is rendered first, 32767
  // is rendered last.
  // There is no getter for blend order in filament, and unlike priority it is
  // too much to track in Impress.
  // //TODO: Add scuba test to validate blend order works correctly
  void SetBlendOrder(uint16_t blend_order,
                     BlendOrderMode mode = BlendOrderMode::kLocal,
                     size_t primitive = 0);

  // Applies changes of all meshes' AABBs and rendering ranges to this
  // MeshRenderer.
  // Note: SetMesh(...) brings all properties of corresponding mesh up-to-date,
  // no need to call this function if changes are introduced only with
  // SetMesh(...).
  void ApplyAllMeshPropertyChanges();

 private:
  static constexpr uint8_t kDefaultPriority = 4;
  static constexpr uint8_t kDefaultChannel = 2;

  enum class HeldPtrType {
    kNone,
    kRawPointer,
    kOwnedPointer,
    kBorrowedPointer
  };

  // Container for primitive data to handle lifetimes automatically.
  struct PrimitiveData {
    // Storage of raw pointer for backwards compatibility with deprecated API.
    Material* raw_material = nullptr;

    OwnedOrBorrowedPtr<Material> owned_or_borrowed_material;

    HeldPtrType held_material_type;

    // Storage of raw pointer for backwards compatibility with deprecated API.
    Mesh* raw_mesh = nullptr;

    OwnedOrBorrowedPtr<Mesh> owned_or_borrowed_mesh;

    HeldPtrType held_mesh_type;

    size_t mesh_index_offset;
    size_t mesh_index_count;
  };

  bool IsOwnedOrBorrowedPtrType(const HeldPtrType& held_ptr_type) const;

  void BuildRenderables(const SetupOptions& options);
  BaseRenderableManager& GetRenderableManager() const;
  filament::RenderableManager::Instance GetInstance() const;

  bool IsWithinCount(size_t primitive_index) const;
  void SetRenderableGeometry(Mesh& mesh, size_t primitive_index);

  // TODO: (broken link) - Remove this once we can query the instance count from
  // the RenderableManager.
  size_t num_instances_ = 1;
  // These are only used if `instancing_mode` is kCpuTransforms.
  // TODO: (broken link) - Remove this once we can query the instance transforms
  // from the InstanceBuffer.
  std::vector<imp::mat4f> instance_transforms_;
  filament::InstanceBuffer* instance_buffer_ = nullptr;

  std::vector<imp::mat4f> bones_;
  std::vector<PrimitiveData> primitives_;
  uint8_t layer_mask_;

  // Filament::RenderableManager has no getter for priority or channel, so track
  // it ourselves.
  uint8_t priority_ = kDefaultPriority;
  uint8_t channel_ = kDefaultChannel;

 public:
  static constexpr bool kRunInEditMode = true;

  using FrustrumCullingMode = FrustumCullingMode;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_MESH_RENDERER_H_
