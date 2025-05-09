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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_ATTRIBUTE_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_ATTRIBUTE_H_

#include <string>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/optional_error.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/typed_id.h"
#include "core/loader/provider/details/vertex_attribute.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"

namespace imp::loader::details::provider_gltf {

using Gltf2Attribute = VertexAttribute;
using Gltf2AttributeMask = VertexAttributeMask;

using AccessorId = TypedId<const imp::gltf::Accessor, int>;

OptionalError GetAttributeType(absl::string_view type, int component_type,
                               schemas::AttributeType *out_type);

const char *GetAttributeName(Gltf2Attribute attribute);

absl::optional<Gltf2Attribute> GetGlTF2VertexAttribute(
    const std::string &attribute_name);

schemas::VertexAttribute GetVertexAttribute(Gltf2Attribute gltf2_attr);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_ATTRIBUTE_H_
