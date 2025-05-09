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

#ifndef THIRD_PARTY_IMPRESS_CORE_RESOURCES_DOWNLOAD_PROGRESS_INFO_H_
#define THIRD_PARTY_IMPRESS_CORE_RESOURCES_DOWNLOAD_PROGRESS_INFO_H_

#include <cstddef>
#include <string>

#include "absl/synchronization/mutex.h"
#include "core/common/robin_map.h"

namespace imp {
namespace resources {
// TODO: Find a better way to surface information about downloads
// in progress. This way of doing it is a bit of a hacky and is pretty
// limited.
// Tracks the number of bytes downloaded and expected for a given URL.
struct EntryProgressInfo {
  size_t downloaded_size;
  // total_size may be kUnknownContentLength if it's unknown.
  size_t total_size;
};
// Tracks all URLs that have been downloaded during the session.
struct DownloadProgressInfo {
  absl::Mutex download_progress_map_mutex;
  imp::RobinMap<std::string, EntryProgressInfo> download_progress_map;
};

}  // namespace resources
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RESOURCES_DOWNLOAD_PROGRESS_INFO_H_
