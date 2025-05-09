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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_PROVIDER_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_PROVIDER_CONSTANTS_H_

#include <cstdint>

namespace imp::loader::details {

// Extensions for relevant file types within loader.
constexpr const char kGltfJsonExtension[] = ".gltf";
constexpr const char kGltfBinaryExtension[] = ".bin";
constexpr const char kGltfBundleExtension[] = ".glb";
constexpr const char kZipArchiveExtension[] = ".zip";
constexpr const char kUsdzExtension[] = ".usdz";

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_PROVIDER_CONSTANTS_H_
