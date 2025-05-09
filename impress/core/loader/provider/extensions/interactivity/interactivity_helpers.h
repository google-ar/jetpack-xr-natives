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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_INTERACTIVITY_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_INTERACTIVITY_HELPERS_H_

#include "core/loader/provider/extensions/interactivity/interactivity.proto.imp.h"

namespace imp::loader::extensions::interactivity {

bool IsValidVariable(const gltf::Interactivity::Graph::Variable& variable);

bool IsValidFlow(const gltf::Interactivity::Graph::Node::Flow& flow);

// Returns the default value for a given type according to KHR_interactivity
// specs:
// https://github.com/KhronosGroup/glTF/blob/interactivity/extensions/2.0/Khronos/KHR_interactivity/Specification.adoc#44-variables
template <typename T>
T GetDefaultValue();

void SetToDefaultValue(gltf::Interactivity::Graph::Variable& variable);

}  // namespace imp::loader::extensions::interactivity

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_INTERACTIVITY_HELPERS_H_
