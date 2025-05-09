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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_PRIMITIVE_SHAPE_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_PRIMITIVE_SHAPE_RENDERER_H_

#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/model/mesh/mesh.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/framework/render/primitive_shape_renderer_state.proto.imp.h"
#include "core/view/framework/render/primitive_shape_type.h"

namespace imp {

// Renders 3D primitive shapes.
class PrimitiveShapeRenderer : public Component {
 public:
  PrimitiveShapeRenderer() {}
  Future<absl::Status> Setup();

  // Setup used when created via an Isf file or AddComponentWithState.
  // This is the preferred method.
  Future<absl::Status> SetupWithState();

  void Cleanup();

  Future<absl::Status> OnIsfStateChanged();

  void OnActiveStatusChanged(bool is_active);

  // Sets the Material used to render the specified primitive.
  //
  // The Primitive Shape Renderer creates a material based on the given
  // definition.
  Future<absl::Status> SetMaterial(MaterialPtr material);
  Future<absl::Status> SetMaterial(Material* material);
  Future<absl::Status> SetMaterial(OwnedMaterialPtr material);
  Future<absl::Status> SetMaterial(BorrowedMaterialPtr material);
  Future<absl::Status> SetMaterial(MaterialDefinition material);

  // Gets the material for the primitive.
  Material* GetMaterial() const;

  // Sets a material parameter on the material.
  Future<absl::Status> SetMaterialParameter(
      MaterialDefinition::Parameter parameter);

  // Gets the mesh for the primitive if set, otherwise returns nullptr.
  // TODO (broken link) Return a BorrowedMeshPtr after migration.
  absl::StatusOr<Mesh*> GetMesh() const;

  // Retrieve information on the type of primitive shape.
  PrimitiveShapeType GetShapeType();

  // Sets the shadow mode that this node can cast onto other nodes.
  // Shadow casting and shadow receiving do not impact each other. For instance,
  // this can cast shadows without receiving shadows.
  void SetShadowCastingMode(MeshRenderer::ShadowMode shadow_mode);

  // Gets the shadow mode that this node can cast onto other nodes.
  // The default value is ShadowMode::kNone.
  MeshRenderer::ShadowMode GetShadowCastingMode() const;

  // Sets the shadow mode that other nodes can cast onto this node.
  // Shadow casting and shadow receiving do not impact each other. For instance,
  // this can receive shadows without casting shadows.
  // If shadow receiving is kNone, this will not receive shadows no matter what
  // the shadow casting setting of other objects is set to.
  void SetShadowReceivingMode(MeshRenderer::ShadowMode shadow_mode);

  // Gets the shadow mode that other nodes can cast onto this node.
  // The default value is ShadowMode::kHardShadows.
  MeshRenderer::ShadowMode GetShadowReceivingMode() const;

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
  absl::Status SetBlendOrder(
      uint16_t blend_order,
      MeshRenderer::BlendOrderMode mode = MeshRenderer::BlendOrderMode::kLocal);

 private:
  PrimitiveShapeType shape_type_;
  PrimitiveShapeRendererState state_;
  ComponentHandle<MeshRenderer> mesh_renderer_;

 public:
  using IsfInfo = IsfInfo<&PrimitiveShapeRenderer::state_>;
  static constexpr bool kRunInEditMode = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_PRIMITIVE_SHAPE_RENDERER_H_
