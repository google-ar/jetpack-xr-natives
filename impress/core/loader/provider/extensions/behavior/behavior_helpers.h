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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_BEHAVIOR_BEHAVIOR_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_BEHAVIOR_BEHAVIOR_HELPERS_H_

#include "core/loader/provider/extensions/behavior/behavior.proto.imp.h"

namespace imp::loader::extensions {

bool IsValidVariable(const gltf::Behavior::Variable& variable);

bool IsValidFlow(const gltf::Behavior::Node::Flow& flow);

}  // namespace imp::loader::extensions

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_BEHAVIOR_BEHAVIOR_HELPERS_H_
