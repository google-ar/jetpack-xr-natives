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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_PARSE_GLTF_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_PARSE_GLTF_H_

#include <vector>

#include "absl/types/optional.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"

namespace imp::loader::details::provider_gltf {

OptionalError TryParseGltf(
    const imp::BufferAccess& primary_resource,
    absl::optional<imp::gltf::imp_proto::Gltf>& out_gltf);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_PARSE_GLTF_H_
