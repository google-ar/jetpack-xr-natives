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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPRITE_SPRITE_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPRITE_SPRITE_RENDERER_H_

#include <optional>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/system.h"
#include "core/render/texture.h"
#include "core/sprite/sprite_renderer_state.proto.imp.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/asset.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// The SpriteRenderer component is used to manage rendering of 2D textures in
// screen space, hence the coordinates of the node the SpriteRenderer is
// attached to should be in (x, y, 0), where x and y are the virtual pixel
// coordinates. Refer to device.h for more details.
//
// The most basic use of the SpriteRenderer would be to load a sprite into it:
// node_->SetWorldPosition(screen.x, screen.y, 0);
// node_->AddComponent<SpriteRenderer>(sprite);
// The sprite can be in the form of an AssetDefinition, an Impress Texture,
// or the url to the texture.
// The sprite can also be loaded from isf if it has a url, an embedded asset
// path, or can be referenced by name from the texture registry.
//
// SpriteRenderer is also meant to be commpletely configured from its State
// proto, where the uv information, texture size, pivot point, and others can
// be specified.
//
// Custom materials can also be loaded by specifying the material asset in the
// `material` parameter in SpriteRendererState. This also means that a texture
// can also be specified as a custom parameter in the material, but note that
// the image parameter in SpriteRendererState must be left empty or it will
// override any textures specified in the material. The blend order will be done
// based on the z-ordering of the sprite for sprites being rendered in the same
// SetPriority() bucket, and blend order fighting issues may occur once the
// limit(32767) is hit.
class SpriteRenderer : public Component {
 public:
  // Component system to handle z-ordering of sprites.
  class System : public ComponentSystem<SpriteRenderer> {
   public:
    explicit System(BaseView* view);

    BorrowedMeshPtr GetQuad();
    void BeforeFirstComponentAdded() override;
    void AfterLastComponentRemoved() override;
    void PostComponentsUpdated(const FrameTime& frame_time) override;

   private:
    OwnedMeshPtr quad_mesh_;
  };

  // Creates a SpriteRenderer. This method is invoked by adding the component by
  // calling AddComponentWithState<SpriteRendererState>(sprite_renderer_state)
  // to specify the image and other params required for the component to work.
  Future<absl::Status> Setup();

  // Creates a SpriteRenderer with the image asset provided.
  Future<absl::Status> Setup(const AssetDefinition& image);

  // Creates a SpriteRenderer by downloading the image provided by the url.
  Future<absl::Status> Setup(absl::string_view url);

  // Creates a SpriteRenderer using the texture provided.
  Future<absl::Status> Setup(const Texture* texture);

  // Creates a SpriteRenderer using the texture provided.
  Future<absl::Status> Setup(TexturePtr texture);

  // *EXPERIMENTAL*
  //
  // Creates a SpriteRenderer using the texture provided.
  //
  // Takes full ownership of texture.  The texture will be destroyed when this
  // SpriteRenderer is.
  Future<absl::Status> Setup(OwnedTexturePtr texture);

  // *EXPERIMENTAL*
  //
  // Creates a SpriteRenderer using the texture provided.
  //
  // The OwnedTexturePtr that texture was borrowed from must not be destroyed
  // until after the SpriteRenderer is destroyed.
  Future<absl::Status> Setup(BorrowedTexturePtr texture);

  // Creates a SpriteRenderer using the material provided
  void Setup(MaterialPtr material);

  // *EXPERIMENTAL*
  //
  // Creates a SpriteRenderer using the material provided
  //
  // Takes full ownership of the material. The material will be destroyed when
  // this SpriteRenderer is.
  void Setup(OwnedMaterialPtr material);

  // Creates a SpriteRenderer using the material asset provided
  void Setup(AssetPtr<MaterialAsset> material);

  void Update(const FrameTime& frame_time);
  void OnActiveStatusChanged(bool is_active);

  void Cleanup();

  // Sets the color of the sprite, which will be multiplied with the texture
  // color. This can usually be used for setting up the alpha of the sprite.
  void SetColor(float4 color);
  float4 GetColor() const;

  // Sets the color of the outline of the sprite. If outline rendering is
  // enabled, a box around the sprite will be rendered in the color.
  void SetOutlineColor(float4 color);
  std::optional<float4> GetOutlineColor() const;

  // Sets the size for the sprite texture in virtual pixels.
  void SetTextureSize(float2 size);
  // Sets the size for the sprite texture in physical pixels.
  // Note that this will fail if the device physical pixel ratio has not yet
  // been set, which will occur during Impress initialization.
  void SetTextureSizePhysicalPixels(float2 physical_pixels);

  // Gets the size for the sprite texture in virtual pixels.
  float2 GetTextureSize() const;
  // Gets the size for the sprite texture in physical pixels.
  // Note that this will fail if the device physical pixel ratio has not yet
  // been set, which will occur during Impress initialization.
  float2 GetTextureSizePhysicalPixels() const;

  // Gets the top-left corner of the texture to display, in UV coords.
  // If one hasn't been set in the state proto, returns the default, (0, 0).
  float2 GetUVOffset() const;
  // Sets the top-left corner of the texture to display, in UV coords.
  void SetUVOffset(float2 uv_offset);

  // Gets the width and height of the texture to display, in UV space.
  // If one hasn't been set in the state proto, returns the default, (1, 1).
  float2 GetUVScale() const;
  // Sets the width and height of the texture to display, in UV space.
  // Note that UV scaling is used to compute the rendered sprite size when
  // physical texture size is set; if you plan to call both SetUVScale() and
  // SetTextureSizePhysicalPixels() during an update, call SetUVScale() first.
  void SetUVScale(float2 uv_scale);

  // Sets the texture to be rendered by the sprite.
  // Note that if the texture cannot change in property here, e.g., if it's
  // switching between a normal and external texture, a new SpriteRenderer is
  // needed.
  void SetTexture(const Texture* texture);

  // Get the pivot point of the sprite, in screen pixels.
  float2 GetPivot() const;

  void SetPivot(float2 pivot);

  // Get the local bounds of the sprite texture on the screen.
  Rect GetLocalBounds() const;

  mat4f GetTextureToScreenMatrix() const;

  SpriteRendererState::RenderSpace GetRenderSpace() const;

  ComponentHandle<MeshRenderer> GetRenderer() const { return mesh_renderer_; }

 private:
  Future<OwnedMaterialPtr> LoadMaterial(const Texture& texture) const;

  // The setup method that's eventually called by all of the public Setup
  // methods.
  void SetupInternal(OwnedMaterialPtr material);

  void UpdateClipSpaceTransform();
  bool IsExternal(const Texture& texture) const;

  float2 texture_size_;
  SpriteRendererState state_;
  NodeHandle mesh_node_;
  ComponentHandle<MeshRenderer> mesh_renderer_;

 public:
  using IsfInfo = IsfInfo<&SpriteRenderer::state_>;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SPRITE_SPRITE_RENDERER_H_
