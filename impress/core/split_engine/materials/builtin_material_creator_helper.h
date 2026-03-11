// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_MATERIAL_CREATOR_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_MATERIAL_CREATOR_HELPER_H_

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// Creates a built-in material spec with default parameters.
flatbuffers::Offset<void> CreateBuiltInMaterialSpecWithDefaultParameters(
    flatbuffers::FlatBufferBuilder& builder,
    android_xr::schemas::BuiltInMaterialSpec spec);

};  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_MATERIAL_CREATOR_HELPER_H_
