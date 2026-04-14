// Copyright 2026 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TEXTURE_PIPELINE_SCHEMA_CONVERSION_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TEXTURE_PIPELINE_SCHEMA_CONVERSION_H_

#include <string>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace imp {
namespace split_engine {

// Converts a TexturePipelineRendererState protobuf message to Split Engine
// TexturePipelineRenderer schema.
absl::StatusOr<
    flatbuffers::Offset<android_xr::schemas::TexturePipelineRenderer>>
TexturePipelineRendererSchemaFromState(
    flatbuffers::FlatBufferBuilder& fbb,
    const imp::TexturePipelineRendererState& state);

// Converts a Split Engine TexturePipelineRenderer schema to a
// TexturePipelineRendererState protobuf message.
absl::StatusOr<TexturePipelineRendererState>
TexturePipelineRendererStateFromSchema(
    BridgeId bridge_id,
    const android_xr::schemas::TexturePipelineRenderer& schema);

}  // namespace split_engine
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_TEXTURE_PIPELINE_SCHEMA_CONVERSION_H_
