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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_H_

#include <sys/types.h>

#include <cstdint>
#include <memory>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/invocable.h"
#include "core/config.h"
#include "core/geometry/shapes/capsule.h"
#include "core/geometry/shapes/sphere.h"
#include "core/lighting/image_based_lighting_types.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_param_value.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/split_engine_mesh_serializer.h"
#include "core/split_engine/split_engine_texture_serializer.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#if IMP_PLATFORM(ANDROID)
#include "core/render/android/android_defines.h"
#include "core/render/android/platform_android_external_texture_surface.h"
#endif
#include "core/render/base_texture_builder.h"

namespace imp {
class Material;
}  // namespace imp

namespace imp::split_engine {

// An interface for serializing the low-level Impress engine state.
// This interface is available on imp::BaseView via the GetSerializer() method.
// Any subsystems that interact with Filament to create renderable content
// should ensure that they are adding that content to the serializer, if it is
// present, to ensure coherence between the local and remote renderable state.
//
// Note: many of these methods take flatbuffer::Offset parameters with the
// expectation that they refer to pre-serialized data of an expected type. That
// expected type is documented on each method that takes such an object. This is
// done to avoid an extra copy during serialization.
class SplitEngineSerializer {
 public:
  virtual ~SplitEngineSerializer() = default;

  // Returns the ID of the given object for serializing pointers.
  static std::uint64_t GetId(const void* ptr);
  // Disassociates the given ID from the object it was associated with.
  static void RemoveId(uint64_t id);

  // Returns the bridge that this serializer uses to serialize data.
  virtual SplitEngineAndroidBridge& GetBridge() = 0;

  // Returns true if the serializer is ready for the next frame, i.e. if the
  // number of in-flight frames is less than the maximum number of allowed
  // in-flight frames.
  virtual bool ReadyForNextFrame() const = 0;

  // Adds a material via material (used for ID) and raw binary material data.
  // TODO: Due to how Filament material versioning works, we need
  // to refactor this API. Our design proposal is that we support two kinds of
  // materials:
  // 1) An enum to a material in Impress generic materials for glTFs. That
  //    same material will be used as loaded by the backend renderer side.
  // 2) A "meta-material" in a protobuffer format that the Impress team will
  //    maintain backwards-compatibility for. This material format will be
  //    converted at runtime into a Filament .mat file and then compiled.
  virtual void AddMaterial(const filament::Material* material,
                           const BufferAccess& data) = 0;
  // Removes a previously-added material from the remote renderer.
  virtual void RemoveMaterial(const filament::Material* material) = 0;
  // Creates a material instance from the given material on the remote renderer.
  // Both material and instance are used to generate IDs for tracking.
  virtual void AddMaterialInstance(
      const filament::Material* material,
      const filament::MaterialInstance* instance) = 0;
  // Serializes an already-duplicated material instance.
  // This will cause the instance to be duplicated on the remote renderer to
  // match the duplication that has already occurred here.
  // Duplicated material instances also duplicate previously-set parameters.
  virtual void DuplicateMaterialInstance(
      const filament::MaterialInstance* instance,
      const filament::MaterialInstance* copy) = 0;
  // Removes a material instance from the remote renderer.
  virtual void RemoveMaterialInstance(
      const filament::MaterialInstance* instance) = 0;
  // Sets a parameter of the given name on the given material.
  // This method handles all valid types of parameters, which are contained in
  // the MaterialParamValue variant.
  virtual void SetMaterialParameter(const filament::MaterialInstance* material,
                                    absl::string_view name,
                                    const MaterialParamValue& value) = 0;
  // Sets a texture parameter of the given name & sampler on the given material.
  virtual void SetMaterialParameter(
      const filament::MaterialInstance* material, absl::string_view name,
      const filament::Texture* texture,
      const filament::TextureSampler& sampler) = 0;

  // Given a texture and a serializer, create a FlatBufferBuilder and
  // serialize the texture into it, before sending to the remote
  // renderer.
  virtual void AddTexture(
      filament::Texture& texture,
      SplitEngineTextureSerializer& split_engine_texture_serializer) = 0;
  virtual void RemoveTexture(filament::Texture& texture) = 0;

  // Given a serializer, creates a FlatBufferBuilder and
  // serialize the mesh into it, before sending to the remote
  // renderer.
  virtual void SerializeMesh(
      SplitEngineMeshSerializer& split_engine_mesh_serializer) = 0;

  // Creates a node for the given entity on the remote renderer.
  virtual void CreateNode(utils::Entity entity) = 0;
  // Destroys a previously-created node on the remote renderer.
  virtual void DestroyNode(utils::Entity entity) = 0;
  // Sets a node to enabled/disabled on the remote renderer.
  virtual void SetEnabled(utils::Entity entity, bool enabled) = 0;
  // Sets the name of a node on the remote renderer.
  virtual void SetName(utils::Entity entity, absl::string_view name) = 0;
  // Sets the parent of a node on the remote renderer.
  virtual void SetParent(utils::Entity entity, utils::Entity parent) = 0;
  // Sets the local transform (relative to parent) on the remote renderer.
  virtual void SetLocalTransform(utils::Entity entity,
                                 const mat4f& transform) = 0;
  // Sets the local transform (relative to parent) on the remote renderer.
  virtual void SetLocalTransform(utils::Entity entity,
                                 const mat4& transform) = 0;
  virtual void AssignUserId(utils::Entity entity, uint32_t user_id) = 0;
  // Creates a texture builder that serializes created textures.
  virtual std::unique_ptr<BaseTextureBuilder> CreateTextureBuilder() = 0;
  // Creates a mesh builder that serializes mesh data.
  virtual std::unique_ptr<BaseMeshBuilder> CreateMeshBuilder() = 0;
#if IMP_PLATFORM(ANDROID)
  // Creates a video source surface to connect external android textures.
  virtual std::unique_ptr<PlatformAndroidExternalTextureSurface>
  CreateAndroidExternalTextureSurface(
      ContentSecurityLevel security_level,
      absl::Span<const SurfaceViewType> view_types) = 0;
#endif

  // ColliderTypes that match the split engine schema.
  // LINT.IfChange
  enum class ColliderType {
    kBoxCollider,
    kMeshCollider,
    kSphereCollider,
    kCapsuleCollider
  };
  // LINT.ThenChange(//depot/google3/third_party/split_engine/schemas/split_engine_data.fbs)
  // Adds a BoxCollider component to the node on the remote renderer.
  virtual void SetBoxCollider(utils::Entity entity, const Box& box,
                              bool enabled) = 0;
  // Adds a SplitEngineMeshCollider component to the node on the remote
  // renderer.
  virtual void SetMeshCollider(utils::Entity entity, bool enabled) = 0;
  // Adds a SphereCollider component to the node on the remote renderer.
  virtual void SetSphereCollider(utils::Entity entity, const Sphere& sphere,
                                 bool enabled) = 0;
  // Adds a CapsuleCollider component to the node on the remote renderer.
  virtual void SetCapsuleCollider(utils::Entity entity, const Capsule& capsule,
                                  bool enabled) = 0;
  // Removes a specific collider from the node on the remote renderer.
  virtual void ClearCollider(utils::Entity entity,
                             ColliderType collider_type) = 0;

  // Creates a generic material on the renderer side based on the given schema
  // and binds it to the given material instance.
  // After the future returns, parameters can be set on the material using
  // SetBuiltInMaterialParameters.
  virtual Future<GenericMaterialPtr> CreateGenericMaterial(
      const GenericMaterialSpec& spec) = 0;

  // This is used to encapsulate the existence of SplitEngineCustomMaterial from
  // the MaterialFactory.
  virtual std::unique_ptr<Material> CreateCustomMaterial(
      std::unique_ptr<Material> material) = 0;

  // Sets the parameters for a built-in material. This call requires that
  // the material has already been created through the Split Engine bridge.
  using SerializeBuiltInMaterialParametersFunc =
      imp::Invocable<flatbuffers::Offset<void>(
          flatbuffers::FlatBufferBuilder& builder)>;
  virtual void SetBuiltInMaterialParameters(
      const filament::MaterialInstance* material,
      android_xr::schemas::BuiltInMaterialParameters type,
      SerializeBuiltInMaterialParametersFunc serialize_func) = 0;

  // Serializes an ImageBasedLightingAsset to the remote renderer.
  virtual void SerializeImageBasedLightingAsset(
      filament::Texture& reflection_texture,
      const SphericalHarmonics& spherical_harmonics,
      const ImageBasedLightingAssetCubemapImages& cubemap_images) = 0;
  virtual void RemoveImageBasedLightingAsset(
      filament::Texture& reflection_texture) = 0;
  // Sets the preferred ImageBasedLightingAsset to use for the environment.
  virtual void SetPreferredEnvironmentIblAsset(
      filament::Texture& reflection_texture, float intensity,
      const float3& tint = float3(1.0f, 1.0f, 1.0f)) = 0;
  // Clears any preferred ImageBasedLightingAsset to use for the environment.
  virtual void ClearPreferredEnvironmentIblAsset() = 0;
  // Destroys MorphTargetBuffer that is given.
  virtual void RemoveMorphTargetBuffer(
      filament::MorphTargetBuffer* morph_target_buffer) = 0;
  // Destroys VertexBuffer that is given.
  virtual void RemoveVertexBuffer(filament::VertexBuffer* vertex_buffer) = 0;
  // Destroys IndexBuffer that is given.
  virtual void RemoveIndexBuffer(filament::IndexBuffer* index_buffer) = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_SERIALIZER_H_
