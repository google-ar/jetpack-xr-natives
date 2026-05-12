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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_MESH_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_MESH_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/collision/mesh_collision_accelerator.h"
#include "core/geometry/shapes/box.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/render/base_renderable_manager.h"
#include "core/render/render_order_constants.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Interface to access the geometry and materials of a Node from a GltfAsset.
//
// The GltfMesh is a Component attached to each rendered node of the model.
// Like all Components it must be stored as a ComponentHandle, not as a raw
// pointer.  Nodes in the model without a GltfMesh are still valid and may
// indicate bones or transforms but will not be directly rendered by the
// GltfRenderer. All the GltfMesh's from a GltfAsset use a single GltfRenderer.
// A GltfMesh may have multiple primitives, each with a GltfMaterial.
class GltfMesh : public Component {
 public:
  // Used to determine how BlendOrder is interpreted. If local, it only applies
  // within a single GltfMesh. If global, applies across all components.
  // Applied on a per-primitive basis.
  enum class BlendOrderMode {
    // Blend order applies to primitives within this component.
    kLocal,
    // Blend order applies to all components.
    kGlobal
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

  // Storing the runtime vertex positions of the meshes.
  struct RuntimeMesh {
    // Pointer to the vertex data for a primitive stored inside the GltfAsset.
    MeshVertexData* mesh_vertex_data;
    // Pointer to the index data for a primitive stored inside the GltfAsset.
    MeshIndexData* mesh_index_data;
    // A copy of the vertex data above that's been transformed based on the
    // current skinning. Null if there is no skinning information for this mesh.
    MeshVertexDataPtr skinned_mesh_vertex_data;
  };

  using PrimitiveType = filament::RenderableManager::PrimitiveType;

  // Transparent renderables will be drawn in order from kMinBlendOrder to
  // kMaxBlendOrder.
  static constexpr uint16_t kMinBlendOrder = 0;
  static constexpr uint16_t kMaxBlendOrder = 0x7FFF;

  GltfMesh();

  void Setup(
      ComponentHandle<GltfRenderer> owner, GltfRenderer::ItemId self,
      std::optional<GltfRenderer::InstanceInfo> instance_info = std::nullopt);
  void Update(const FrameTime& frame_time);
  void Cleanup();

  size_t GetPrimitiveCount() const;

  // Returns the material currently used for rendering.
  //
  // If a material override has been set this will return the override instead
  // of the original Material.
  Material* GetMaterial(size_t primitive_index = 0) const;

  // Returns the material override or null if there is none.
  Material* GetMaterialOverride(size_t primitive_index = 0) const;

  // Same as above, but does not take ownership of the `material`.
  ABSL_DEPRECATED(
      "Use imp::BorrowedMaterialPtr overload instead. See "
      "(broken link).")
  void SetMaterialOverride(Material* material, size_t primitive_index = 0);

  // Overrides the material used on a primitive in this GltfMesh.
  //
  // This changes the underlying assignment for rendering, but does
  // not change the material assigned in the underlying GltfAsset. If material
  // is null, the original material from the models data will be used.
  void SetMaterialOverride(OwnedMaterialPtr material,
                           size_t primitive_index = 0);

  // Same as above, but does not take ownership of the `material`.
  void SetMaterialOverride(BorrowedMaterialPtr material,
                           size_t primitive_index = 0);

  // Sets the rendering bounds in local space for this node.
  //
  // The bounds are used to cull the mesh when it isn't visible to
  // the camera. By default, this is the union of the bounds for each primitive
  // in this mesh.
  void SetLocalBounds(const Box& local_bounds);

  // Gets the rendering bounds in local space for this node.
  //
  // The bounds are used to cull the mesh when it isn't visible to
  // the camera. By default, this is the union of the bounds for each primitive
  // in this mesh.
  const Box& GetLocalBounds() const;

  // Gets the rendering bounds in world space for this node.
  //
  // The bounds are used to cull the mesh when it isn't visible to
  // the camera. By default, this is the union of the bounds for each primitive
  // in this mesh.
  Box GetWorldBounds() const;

  // Returns the name assigned to this mesh from the glTF data.
  // Mesh names may be empty and are not unique.
  const std::string& GetName() const;

  // Returns the primitive type to be rendered.
  PrimitiveType GetPrimitiveType(size_t primitive_index = 0) const;

  // Returns number of morph targets.
  size_t GetMorphTargetCount() const;

  // Returns the current morph target weights.
  std::vector<float> GetMorphTargetWeights() const;

  // Returns a certain morph target's weight.
  float GetMorphTargetWeight(size_t index) const;

  // Sets morph target weights on the Filament side for real-time rendering.
  // Sparse accessors for morph targets are currently not implemented.
  void SetMorphTargetWeights(const std::vector<float>& weights);

  // Sets an individual morph target's weight on the Filament side for
  // real-time rendering.
  void SetMorphTargetWeight(size_t index, float weight);

  // Sets the blend order, used to provide a limited amount of control over the
  // draw order of this node. Only works in transparent blend mode.
  //
  // BlendOrderMode determines if the order is interpreted locally, relative to
  // this component only, or globally across all components.
  //
  // The order is clamped to the range [0..32767]. 0 is rendered first, 32767
  // is rendered last.
  void SetBlendOrder(uint16_t blend_order,
                     BlendOrderMode mode = BlendOrderMode::kLocal,
                     size_t primitive = 0);

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
  // The default value is 0.
  uint8_t GetChannel() const;

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

  // Enables/Disables the fog.
  void SetFogEnabled(bool enable);

  // Returns true if the fog is enabled.
  bool GetFogEnabled() const;

  // Access the vertex and index information of primitives.
  absl::Span<const MeshVertexAndIndexData> GetMeshData() const;

  // Returns the Bvh for the mesh to accelerate collision queries, if any.
  const MeshCollisionAccelerator* GetMeshCollisionAccelerator() const;

  // This is used only when the data for the vertices is also stored outside the
  // GPU to apply skinning on the CPU.
  bool IsSkinned() const;

  // Get the original glTF node index this GltfMesh is representing
  uint64_t GetOriginalGltfIndex() const;

  // Get the original glTF mesh index this GltfMesh is representing
  int16_t GetOriginalGltfMeshIndex() const;

  // Update skinned mesh with new skinning data.
  void UpdateSkinnedMesh();

  // Returns the transforms used for instancing the mesh, if any.
  const std::vector<mat4f>& GetInstanceTransforms() const;

  // Returns the GltfRenderer that created this GltfMesh.
  //
  // The GltfRenderer is on the root node of the glTF hierarchy.
  ComponentHandle<GltfRenderer> GetGltfRenderer() const;

 private:
  // The GltfRenderer that owns us.  Our lifetime is bound to our owners'.
  ComponentHandle<GltfRenderer> owner_;
  // The id (index) that corresponds to the node we're attached to;
  // i.e. owner_->nodes_[self_] == GetNode()
  GltfRenderer::ItemId self_;
  // We cache the number of primitives we have so we don't have to depend on
  // owner_ existing at Cleanup time.
  size_t primitive_count_;

  // Runtime mesh data.
  std::vector<RuntimeMesh> runtime_meshes_;
  // Per-triangle mesh, matches GetMeshData() return type for downstream usage.
  std::vector<MeshVertexAndIndexData> primitive_mesh_data_;
  bool skinned_mesh_data_updated_ = false;

  std::vector<mat4f> instance_transforms_;
  filament::InstanceBuffer* instance_buffer_ = nullptr;

  // Filament::RenderableManager has no getter for priority or channel, so track
  // it ourselves.
  uint8_t priority_ = kDefaultPriority;
  uint8_t channel_ = kDefaultChannel;
  BaseRenderableManager& GetRenderableManager() const;
  filament::RenderableManager::Instance GetInstance() const;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_GLTF_MESH_H_
