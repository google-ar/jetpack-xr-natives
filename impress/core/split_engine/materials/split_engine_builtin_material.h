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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_BUILTIN_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_BUILTIN_MATERIAL_H_

#include <memory>
#include <variant>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/common/robin_set.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

using PlaceholderOrBuiltInMaterialPtr =
    std::variant<OwnedMaterialPtr, BuiltInMaterialPtr>;

// A base class for Split Engine built-in materials on the app-side. This class
// encapsulates the boilerplate for serializing material parameters.
class SplitEngineBuiltinMaterial {
 public:
  static Future<OwnedMaterialPtr> CreatePlaceholderMaterial(BaseView& view);

  static Future<PlaceholderOrBuiltInMaterialPtr> RequestBuiltInMaterial(
      BaseView& view, std::unique_ptr<flatbuffers::FlatBufferBuilder> fbb,
      android_xr::schemas::BuiltInMaterialSpec material_type,
      flatbuffers::Offset<void> spec);

  // Creates a Split Engine built-in material with the given parameters schema
  // type and material instance.
  SplitEngineBuiltinMaterial(
      BaseView& view,
      android_xr::schemas::BuiltInMaterialParameters parameters_type,
      PlaceholderOrBuiltInMaterialPtr material);

  virtual ~SplitEngineBuiltinMaterial();

  // Returns the app-side material instance to use to attach to renderables.
  BorrowedMaterialPtr GetMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Triggers serialization of the material parameters to the built-in material.
  void UpdateParameters() const;

 protected:
  // Subclasses should implement this method to serialize their parameters.
  // This method will be called each frame, and should only return a non-empty
  // offset if the parameters have changed since the last frame.
  virtual flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      BuiltInTextureParameterCreator& texture_parameter_creator) const = 0;

  // Marks the parameters as dirty, which will cause them to be serialized
  // again at the end of the frame.
  void MarkParametersDirty(bool dirty = true) const;
  bool AreParametersDirty() const;

  // Destroys the material. This should be called by subclasses destructors
  // prior to destroying any owned textures to ensure proper destruction order.
  void Cleanup();

 private:
  BaseView& view_;
  android_xr::schemas::BuiltInMaterialParameters parameters_type_;
  PlaceholderOrBuiltInMaterialPtr material_;
  bool cleanup_called_ = false;
};

// An updater that calls Update() on all SplitEngineMaterials in the view.
class SplitEngineMaterialUpdater
    : public UpdateSystem::Updater<SplitEngineMaterialUpdater> {
 public:
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;

  explicit SplitEngineMaterialUpdater(BaseView& view);

  void Update(const FrameTime& frame_time) override;
  void MarkParametersDirty(const SplitEngineBuiltinMaterial* material,
                           bool dirty = true);
  bool AreParametersDirty(const SplitEngineBuiltinMaterial* material) const;
  void RemoveMaterial(SplitEngineBuiltinMaterial* material);

 private:
  // Set of SplitEngineMaterials whose parameters have changed this frame.
  RobinSet<const SplitEngineBuiltinMaterial*> dirty_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_BUILTIN_MATERIAL_H_
