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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_GLTF_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_GLTF_PROVIDER_H_

#include "absl/status/statusor.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/details/provider_details_common.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"

namespace imp::loader::details {
namespace provider_gltf {
class ParsedGltf;
}

// Abstract class for handling of glTF/glb files. This allows us to
// incrementally replace it and A/B test against a faster and more secure
// version.
class GltfProvider {
 public:
  virtual ~GltfProvider() {}

  // Attempt to parse the gltf file, retrieving a tinygltf::Model.  This can be
  // used in subsequent calls to TryLoadGltf.
  virtual OptionalError TryParseGltf(LoaderState* state) = 0;

  virtual bool IsParsed() = 0;

  // Check a successful parsed model for pending resources.  This allows us to
  // flag missing resources encountered during a successful parsing, and update
  // the tracking information that allows us to maintain those links when we
  // attempt to load.
  virtual bool HasPendingResources(LoaderState* state) = 0;

  // Attempt a single gltf load.
  virtual absl::StatusOr<FlatBufferAccess<schemas::LoadedModel>> TryLoadGltf(
      LoaderState* state) = 0;

  // Returns the parsed gltf for testing.
  virtual const provider_gltf::ParsedGltf* GetParsedGltf() const = 0;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_GLTF_PROVIDER_H_
