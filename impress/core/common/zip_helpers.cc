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

#include "core/common/zip_helpers.h"

#include <array>
#include <set>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "core/common/platform_helpers.h"
#include "zlib/contrib/minizip/unzip.h"
// unzip.h must be included before ioapi.h for types to be defined correctly.
#include "zlib/contrib/minizip/ioapi.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace {

OptionalError OpenZip(const BufferAccess& zip_access, unzFile* out_zip_file) {
  zlib_filefunc_def def;
  voidpf mem_zipfile = mem_simple_create_file(
      &def, const_cast<uint8_t*>(zip_access.Data()), zip_access.Size());

  *out_zip_file = unzAttach(mem_zipfile, &def);
  if (*out_zip_file == nullptr) {
    free(mem_zipfile);
    return Error("Failed to attach to zip (%llu bytes)", zip_access.Size());
  }
  return NoError();
}

}  // namespace

OptionalError GetFilenamesFromZip(const BufferAccess& zip_access,
                                  std::vector<std::string>* out_filenames) {
  unzFile zip_file;
  MP_RETURN_IF_ERROR(OpenZip(zip_access, &zip_file));

  auto filename_storage = std::array<char, 1024>();
  unz_file_info info;
  do {
    if (unzGetCurrentFileInfo(zip_file, &info, filename_storage.data(),
                              filename_storage.size(), nullptr, 0, nullptr,
                              0) == UNZ_OK) {
      if (info.size_filename > filename_storage.size()) {
        unzClose(zip_file);
        return Error("Invalid zip file");
      }

      out_filenames->push_back({filename_storage.data(), info.size_filename});
    }
  } while (unzGoToNextFile(zip_file) == UNZ_OK);

  unzClose(zip_file);
  return NoError();
}

absl::StatusOr<std::vector<ZipFile>> GetFilesFromZip(
    const BufferAccess& zip_access, const std::set<std::string>& filenames) {
  std::vector<ZipFile> result;
  if (filenames.empty()) {
    return result;
  }

  unzFile zip_file;
  MP_RETURN_IF_ERROR(OpenZip(zip_access, &zip_file));

  unz_file_info file_info;

  // Scrub to the file and open it (if we can).

  const int kIgnoreCase = 2;  // see google3/third_party/minizip/unzip.h:159
  for (const std::string& filename : filenames) {
    if (unzLocateFile(zip_file, filename.c_str(), kIgnoreCase) != UNZ_OK ||
        unzGetCurrentFileInfo(zip_file, &file_info, nullptr, 0, nullptr, 0,
                              nullptr, 0) != UNZ_OK ||
        file_info.uncompressed_size <= 0 ||
        unzOpenCurrentFile(zip_file) != UNZ_OK) {
      unzClose(zip_file);
      return Error("Failed to fetch file '%s' from archive", filename);
    }
    // Read it in (if we can)

    BufferAccess file_buffer;
    if (unzReadCurrentFile(
            zip_file,
            BufferAccess::Create(file_info.uncompressed_size, &file_buffer),
            file_info.uncompressed_size) !=
        static_cast<int>(file_info.uncompressed_size)) {
      unzClose(zip_file);
      return Error("Failed to read all %llu bytes of file '%s' from archive",
                   file_info.uncompressed_size, filename);
    }
    result.emplace_back(filename, std::move(file_buffer));
  }
  unzClose(zip_file);

  if (result.size() != filenames.size()) {
    return absl::InternalError("Unable to find all files.");
  }

  return result;
}

}  // namespace imp
