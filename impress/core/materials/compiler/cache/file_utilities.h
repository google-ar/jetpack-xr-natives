/*
 * Copyright 2025 Google LLC
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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_CACHE_FILE_UTILITIES_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_CACHE_FILE_UTILITIES_H_

#include <sys/stat.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "core/common/context.h"

namespace imp {

// Interface for file system utilities to allow for mocking in tests.
// NOTE: We use std::string instead of absl::string_view because sys/stat
// methods require null terminated strings.
class FileUtilities {
 public:
  virtual ~FileUtilities() = default;

  // Returns the directory to use for caching on each platform without the
  // trailing slash.
  virtual std::string GetCacheDir() = 0;
  virtual absl::Status MkdirRecursive(std::string dir) = 0;
  virtual int64_t GetFileSize(const std::string& file_path) = 0;
  virtual bool Exists(const std::string& path) = 0;
  virtual absl::Status ReadFile(const std::string& file_path,
                                std::vector<uint8_t>& buffer) = 0;
  virtual absl::Status WriteFile(const std::string& file_path,
                                 const std::vector<uint8_t>& buffer) = 0;
  virtual void ForEachFileInDir(
      const std::string& dir_path, const std::string& suffix,
      const std::function<void(const std::string&, const struct stat&)>&
          callback) = 0;
  virtual absl::Status RecursivelyDelete(const std::string& path) = 0;
  virtual std::string Stem(const std::string& path) = 0;
};

// Creates a default implementation of FileUtilities.
std::unique_ptr<FileUtilities> CreateFileUtilities(const Context& context);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_CACHE_FILE_UTILITIES_H_
