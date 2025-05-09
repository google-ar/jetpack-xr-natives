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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_ZIP_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_ZIP_HELPERS_H_

#include <set>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

// This denotes a file extracted from zip.
struct ZipFile {
  ZipFile(std::string filename_in, BufferAccess access_in)
      : filename(std::move(filename_in)), access(std::move(access_in)) {}
  std::string filename;
  BufferAccess access;
};

OptionalError GetFilenamesFromZip(const BufferAccess& zip_access,
                                  std::vector<std::string>* out_filenames);

absl::StatusOr<std::vector<ZipFile>> GetFilesFromZip(
    const BufferAccess& zip_access, const std::set<std::string>& filenames);

inline OptionalError GetFileFromZip(const BufferAccess& zip_access,
                                    const std::string& filename,
                                    BufferAccess* out_access) {
  MP_ASSIGN_OR_RETURN(auto file_buffers, GetFilesFromZip(zip_access, {filename}));

  *out_access = std::move(file_buffers[0].access);

  return NoError();
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_ZIP_HELPERS_H_
