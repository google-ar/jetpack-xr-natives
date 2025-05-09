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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_BEHAVIOR_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_BEHAVIOR_H_

#include <memory>

#include "flatbuffers/flatbuffer_builder.h"
#include "core/loader/provider/extensions/behavior/loader_extension.h"
#include "core/loader/provider/extensions/behavior/model_creator_extension.h"
#include "core/proto/json_message_visitor.h"

namespace imp::loader::extensions {

// This class adds functionality to the glTF loader for parsing the glTF
// behavior extension (KHR_behavior)
class Behavior {
 public:
  // Registers all OnVisit and OnPostVisit handlers to the JsonMessageVisitor.
  // This allows the Behavior extension to register custom visitors to various
  // messages in the glTF Behavior proto.
  virtual void AddHooks(proto::JsonMessageVisitor& json_message_visitor) = 0;

  virtual ~Behavior() = default;
};

std::unique_ptr<Behavior> CreateBehaviorGltfExtension();

// Creates the Behavior extension for handling KHR_interactivity content in
// the glTF loader.
std::unique_ptr<details::BehaviorLoaderExtension> CreateBehaviorLoaderExtension(
    flatbuffers::FlatBufferBuilder& fbb);

// Creates the Behavior extension for handling KHR_interactivity content in
// the ModelCreator.
std::unique_ptr<details::BehaviorModelCreatorExtension>
CreateBehaviorModelCreatorExtension();

}  // namespace imp::loader::extensions

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_BEHAVIOR_H_
