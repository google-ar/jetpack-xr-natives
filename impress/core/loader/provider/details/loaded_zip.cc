// Copyright 2024 Google LLC
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

#include "core/loader/provider/details/loaded_zip.h"

#include "absl/algorithm/container.h"
#include "core/common/zip_helpers.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {

OptionalError TryAddMissingResources(const LoadedZip* loaded_zip,
                                     LoaderState* state) {
  std::vector<std::string> found_files;
  for (auto it : state->missing_resource_name_from_path_) {
    auto found_it = absl::c_find_if(
        loaded_zip->filenames,
        [&it](const std::string& filename) { return filename == it.first; });

    if (found_it != loaded_zip->filenames.end()) {
      BufferAccess found_access;
      MP_RETURN_IF_ERROR(
          GetFileFromZip(loaded_zip->zip_archive, *found_it, &found_access));
      state->AddResource(it.second, std::move(found_access));
      found_files.push_back(it.first);
    }
  }
  for (auto& found_file : found_files) {
    state->missing_resource_name_from_path_.erase(found_file);
  }
  return NoError();
}

}  // namespace imp::loader::details
