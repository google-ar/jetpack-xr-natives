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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_PROVIDER_GLTF_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_PROVIDER_GLTF_H_

#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/details/gltf_provider.h"
#include "core/loader/provider/details/provider_details_common.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"

namespace imp::gltf::imp_proto {
class Gltf;
}  // namespace imp::gltf::imp_proto

namespace imp::loader::details::provider_gltf {

std::unique_ptr<GltfProvider> CreateGltfProvider();

absl::Status ResolveResources(
    absl::string_view directory,
    tsl::robin_map<std::string, BufferAccess>& resources,
    tsl::robin_map<std::string, std::string>& missing_resource_name_from_path,
    imp::gltf::imp_proto::Gltf& gltf, std::vector<BufferAccess>& owned);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_PROVIDER_GLTF_H_
