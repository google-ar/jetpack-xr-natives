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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_MODEL_CREATOR_EXTENSION_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_MODEL_CREATOR_EXTENSION_H_

#include "absl/status/statusor.h"
#include "core/loader/provider/extensions/interactivity/schemas/interactivity_generated.h"
#include "core/model/model_data.h"

namespace imp::loader::details {

// An extension for handling Interactivity logic in the ModelCreator.
class InteractivityModelCreatorExtension {
 public:
  // Deserializes the InteractivityData from the provided Interactivity
  // flatbuffer.
  virtual absl::StatusOr<model::ModelData::InteractivityData>
  DeserializeInteractivityData(const schemas::Interactivity *behavior) = 0;

  virtual ~InteractivityModelCreatorExtension() = default;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_MODEL_CREATOR_EXTENSION_H_
