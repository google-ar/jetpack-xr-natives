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

#include "core/assets/gltf/object_model/pointer_declarations/core_pointers.h"

#include <memory>
#include <vector>

#include "core/assets/gltf/object_model/pointer_declarations/materials.h"
#include "core/assets/gltf/object_model/pointer_declarations/meshes.h"
#include "core/assets/gltf/object_model/pointer_declarations/nodes.h"
#include "core/assets/gltf/object_model/pointer_declarations/skins.h"
#include "core/assets/gltf/object_model/property_pointer.h"

namespace imp::gltf {

std::vector<std::unique_ptr<PropertyPointer::PointerDeclaration>>
GetCorePointerDeclarations() {
  std::vector<std::unique_ptr<PropertyPointer::PointerDeclaration>>
      declarations;

  // nodes
  declarations.push_back(
      std::make_unique<NodesGlobalMatrixPointerDeclaration>());
  declarations.push_back(std::make_unique<NodesMatrixPointerDeclaration>());
  declarations.push_back(std::make_unique<NodesRotationPointerDeclaration>());
  declarations.push_back(std::make_unique<NodesScalePointerDeclaration>());
  declarations.push_back(
      std::make_unique<NodesTranslationPointerDeclaration>());
  declarations.push_back(std::make_unique<NodesLengthPointerDeclaration>());
  declarations.push_back(
      std::make_unique<NodesChildrenLengthPointerDeclaration>());
  declarations.push_back(std::make_unique<NodesParentPointerDeclaration>());
  declarations.push_back(std::make_unique<NodesMeshPointerDeclaration>());
  declarations.push_back(
      std::make_unique<NodesWeightsLengthPointerDeclaration>());

  // materials
  declarations.push_back(
      std::make_unique<MaterialsAlphaCutoffPointerDeclaration>());
  declarations.push_back(
      std::make_unique<MaterialsEmissiveFactorPointerDeclaration>());
  declarations.push_back(
      std::make_unique<MaterialsNormalTextureScalePointerDeclaration>());
  declarations.push_back(
      std::make_unique<MaterialsOcclusionTextureStrengthPointerDeclaration>());
  declarations.push_back(
      std::make_unique<
          MaterialsPbrMetallicRoughnessBaseColorFactorPointerDeclaration>());
  declarations.push_back(
      std::make_unique<
          MaterialsPbrMetallicRoughnessMetallicFactorPointerDeclaration>());
  declarations.push_back(
      std::make_unique<
          MaterialsPbrMetallicRoughnessRoughnessFactorPointerDeclaration>());
  declarations.push_back(std::make_unique<MaterialsLengthPointerDeclaration>());

  // skins
  declarations.push_back(
      std::make_unique<SkinsJointsLengthPointerDeclaration>());
  declarations.push_back(std::make_unique<SkinsJointNodePointerDeclaration>());
  declarations.push_back(std::make_unique<SkinsSkeletonPointerDeclaration>());
  declarations.push_back(std::make_unique<SkinsLengthPointerDeclaration>());

  // meshes
  declarations.push_back(std::make_unique<MeshesLengthPointerDeclaration>());

  return declarations;
}

}  // namespace imp::gltf
