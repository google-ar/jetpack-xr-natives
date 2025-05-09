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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_MESHOPT_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_MESHOPT_H_

#include "absl/status/statusor.h"
#include "core/common/buffer_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"

namespace imp::loader::extensions {

// Resolves any BufferViews with the EXT_meshopt_compression extension. Returns
// a BufferAccess containing decoded data pointed to by the (modified) gltf.
absl::StatusOr<BufferAccess> ResolveMeshOpt(imp::gltf::Gltf* gltf);

}  // namespace imp::loader::extensions

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_EXTENSIONS_GLTF_EXTENSION_MESHOPT_H_
