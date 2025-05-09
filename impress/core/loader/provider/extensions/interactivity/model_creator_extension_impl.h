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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_MODEL_CREATOR_EXTENSION_IMPL_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_MODEL_CREATOR_EXTENSION_IMPL_H_

#include "absl/status/statusor.h"
#include "core/loader/provider/extensions/interactivity/model_creator_extension.h"
#include "core/loader/provider/extensions/interactivity/schemas/interactivity_generated.h"
#include "core/model/model_data.h"

namespace imp::loader::details {

class InteractivityModelCreatorExtensionImpl
    : public InteractivityModelCreatorExtension {
 public:
  absl::StatusOr<model::ModelData::InteractivityData>
  DeserializeInteractivityData(
      const schemas::Interactivity *interactivity) override;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_INTERACTIVITY_MODEL_CREATOR_EXTENSION_IMPL_H_
