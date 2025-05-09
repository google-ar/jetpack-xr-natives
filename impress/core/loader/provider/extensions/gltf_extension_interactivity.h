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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_INTERACTIVITY_BEHAVIOR_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_INTERACTIVITY_BEHAVIOR_H_

#include <memory>

#include "flatbuffers/flatbuffer_builder.h"
#include "core/loader/provider/extensions/interactivity/loader_extension.h"
#include "core/loader/provider/extensions/interactivity/model_creator_extension.h"
#include "core/proto/json_message_visitor.h"

namespace imp::loader::extensions {

// This class adds functionality to the glTF loader for parsing the glTF
// interactivity extension (KHR_interactivity)
class Interactivity {
 public:
  // Registers all OnVisit and OnPostVisit handlers to the JsonMessageVisitor.
  // This allows the Interactivity extension to register custom visitors to
  // various messages in the glTF Interactivity proto.
  virtual void AddHooks(proto::JsonMessageVisitor& json_message_visitor) = 0;

  virtual ~Interactivity() = default;
};

std::unique_ptr<Interactivity> CreateInteractivityGltfExtension();

// Creates the Interactivity extension for handling KHR_interactivity content in
// the glTF loader.
std::unique_ptr<details::InteractivityLoaderExtension>
CreateInteractivityLoaderExtension(flatbuffers::FlatBufferBuilder& fbb);

// Creates the Interactivity extension for handling KHR_interactivity content in
// the ModelCreator.
std::unique_ptr<details::InteractivityModelCreatorExtension>
CreateInteractivityModelCreatorExtension();

}  // namespace imp::loader::extensions

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_INTERACTIVITY_BEHAVIOR_H_
