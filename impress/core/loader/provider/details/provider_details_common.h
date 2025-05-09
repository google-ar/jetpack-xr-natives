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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_PROVIDER_DETAILS_COMMON_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_PROVIDER_DETAILS_COMMON_H_

#include <string>

#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/common/buffer_access.h"
#include "core/loader/loader_options.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp::loader::details {

struct MissingResource {
  const std::string name;
  const std::string path;
};
using MissingResources = std::vector<MissingResource>;
using ResourceMap = tsl::robin_map<std::string, BufferAccess>;

// Embedded Images - resides inside a glb, or embedded gltf.
struct EmbeddedImage {
  BufferAccess access;
};
// Linked Images - refers to a resource by name.
struct LinkedImage {
  std::string resource_name;
};
using TrackedImage = absl::variant<EmbeddedImage, LinkedImage>;

//
class LoaderState {
 public:
  // Loader is not copyable.
  LoaderState(const LoaderState&) = delete;
  LoaderState& operator=(const LoaderState& rhs) = delete;

  LoaderState(absl::string_view directory, absl::string_view basename,
              absl::string_view extension, BufferAccess&& primary_resource,
              LoaderOptions options = {});

  const std::string directory_;
  const std::string basename_;
  const std::string extension_;
  const BufferAccess primary_resource_;
  const LoaderOptions options_;
  // Not const - state can accumulate resources over time.
  ResourceMap resources_;
  // Not const - state can accumulate tracked images over time.
  std::vector<TrackedImage> tracked_images_;
  // Not const - state tracks missing resources as they're discovered.
  tsl::robin_map<std::string, std::string> missing_resource_name_from_path_;

  void AddResource(absl::string_view path, BufferAccess&& access);
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_DETAILS_PROVIDER_DETAILS_COMMON_H_
