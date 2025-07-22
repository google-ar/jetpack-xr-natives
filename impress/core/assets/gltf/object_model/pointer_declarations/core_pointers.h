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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_CORE_POINTERS_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_CORE_POINTERS_H_

#include <memory>
#include <vector>

#include "core/assets/gltf/object_model/property_pointer.h"

namespace imp::gltf {

// Returns a vector of all core pointer declarations.
// For more information on core pointers, see:
// https://github.com/KhronosGroup/glTF/blob/main/specification/2.0/ObjectModel.adoc#core-pointers
std::vector<std::unique_ptr<PropertyPointer::PointerDeclaration>>
GetCorePointerDeclarations();

// TODO: Add the rest of the core pointer declarations.

}  // namespace imp::gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_CORE_POINTERS_H_
