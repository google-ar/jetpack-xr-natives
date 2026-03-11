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
#include "core/materials/compiler/cache/file_utilities.h"

#include <dirent.h>
#include <errno.h>
#include <sys/stat.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <ios>
#include <memory>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "core/common/context.h"
#include "core/config.h"
#if IMP_PLATFORM(ANDROID)
#include "core/materials/compiler/cache/file_utilities_jni.h"
#endif

namespace imp {
namespace {

class FileUtilitiesImpl : public FileUtilities {
 public:
  FileUtilitiesImpl(const Context& context)
#if IMP_PLATFORM(ANDROID)
      : context_(context)
#endif
  {
  }

  std::string GetCacheDir() override {
    std::string path;
#if IMP_PLATFORM(ANDROID)
    JavaFileUtilities java_file_utils(context_);
    path = java_file_utils.GetCacheDirectory();
#elif IMP_PLATFORM(LINUX)
    const char* cache_home = getenv("XDG_CACHE_HOME");
    if (cache_home && cache_home[0] != '\0') {
      path = cache_home;
    } else {
      const char* home = getenv("HOME");
      if (home && home[0] != '\0') {
        path = std::string(home) + "/.cache";
      } else {
        path = "/tmp";
      }
    }
#elif IMP_PLATFORM(MACOS) || IMP_PLATFORM(IOS)
    const char* home = getenv("HOME");
    if (home && home[0] != '\0') {
      path = std::string(home) + "/Library/Caches";
    } else {
      path = "/tmp";
    }
#else
    path = "/tmp";
#endif
    return path + "/impress";
  }

  absl::Status MkdirRecursive(std::string dir) override {
    if (dir.empty()) {
      return absl::InvalidArgumentError("Directory path is empty.");
    }
    if (absl::EndsWith(dir, "/")) {
      dir.pop_back();
      if (dir.empty()) {
        return absl::OkStatus();  // root dir or end of path found
      }
    }

    struct stat st;
    if (stat(dir.c_str(), &st) == 0) {
      if (S_ISDIR(st.st_mode)) {
        return absl::OkStatus();
      } else {
        return absl::FailedPreconditionError(
            absl::StrCat(dir, " exists but is not a directory."));
      }
    }
    size_t pos = dir.rfind('/');
    if (pos != std::string::npos) {
      std::string parent_dir = dir.substr(0, pos);
      if (!parent_dir.empty()) {
        if (absl::Status status = MkdirRecursive(parent_dir); !status.ok()) {
          return status;
        }
      }
    }
    // Create a directory where we have permission to read, write, and execute.
    // 0755 = Owner rwx, Group rx, Others rx
    if (mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
      return absl::InternalError(
          absl::StrCat("mkdir failed for ", dir, " with errno: ", errno));
    }
    return absl::OkStatus();
  }

  int64_t GetFileSize(const std::string& file_path) override {
    struct stat stat_buf;
    if (stat(file_path.c_str(), &stat_buf) == 0) {
      return stat_buf.st_size;
    }
    return 0;
  }

  bool Exists(const std::string& path) override {
    struct stat stat_buf;
    return stat(path.c_str(), &stat_buf) == 0;
  }

  absl::Status ReadFile(const std::string& file_path,
                        std::vector<uint8_t>& buffer) override {
    std::ifstream file(file_path.c_str(), std::ios::binary | std::ios::ate);
    if (!file) {
      return absl::InternalError(
          absl::StrCat("Failed to open file for reading: ", file_path));
    }

    std::ifstream::pos_type file_size = file.tellg();
    buffer.resize(file_size);

    file.seekg(0, std::ios::beg);
    file.read(reinterpret_cast<char*>(buffer.data()), file_size);
    if (file.fail()) {
      file.close();
      return absl::InternalError(
          absl::StrCat("Failed to read file: ", file_path));
    }
    file.close();
    return absl::OkStatus();
  }

  absl::Status WriteFile(const std::string& file_path,
                         const std::vector<uint8_t>& buffer) override {
    std::ofstream file(file_path.c_str(), std::ios::binary | std::ios::trunc);
    if (!file) {
      return absl::InternalError(
          absl::StrCat("Failed to open file for writing: ", file_path));
    }
    file.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    if (file.fail()) {
      file.close();
      return absl::InternalError(
          absl::StrCat("Failed to write to file: ", file_path));
    }
    file.close();
    return absl::OkStatus();
  }

  void ForEachFileInDir(
      const std::string& dir_path, const std::string& suffix,
      const std::function<void(const std::string&, const struct stat&)>&
          callback) override {
    DIR* dir = opendir(dir_path.c_str());
    if (!dir) {
      return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      if (entry->d_type == DT_REG) {
        std::string filename = entry->d_name;
        if (!absl::EndsWith(filename, suffix)) {
          continue;
        }
        std::string full_path = std::string(dir_path) + "/" + filename;
        struct stat statbuf;
        if (stat(full_path.c_str(), &statbuf) == 0) {
          callback(full_path, statbuf);
        }
      }
    }
    closedir(dir);
  }

  absl::Status RecursivelyDelete(const std::string& path) override {
    if (!Exists(path)) {
      return absl::OkStatus();
    }
    DIR* dir = opendir(path.c_str());
    if (!dir) {
      if (remove(path.c_str()) != 0) {
        return absl::InternalError(
            absl::StrCat("Failed to delete file: ", path));
      }
      return absl::OkStatus();
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
      if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
        continue;
      }
      std::string full_path = std::string(path) + "/" + entry->d_name;
      if (absl::Status status = RecursivelyDelete(full_path); !status.ok()) {
        closedir(dir);
        return status;
      }
    }
    closedir(dir);
    if (rmdir(path.c_str()) != 0) {
      return absl::InternalError(
          absl::StrCat("Failed to delete directory: ", path));
    }
    return absl::OkStatus();
  }

  std::string Stem(const std::string& path) override {
    std::string basename = path;
    size_t pos = path.rfind('/');
    if (pos != std::string::npos) {
      basename = path.substr(pos + 1);
    }
    pos = basename.rfind('.');
    if (pos == std::string::npos) {
      return basename;
    }
    return basename.substr(0, pos);
  }

 private:
#if IMP_PLATFORM(ANDROID)
  const Context& context_;
#endif
};

}  // namespace

std::unique_ptr<FileUtilities> CreateFileUtilities(const Context& context) {
  return std::make_unique<FileUtilitiesImpl>(context);
}

}  // namespace imp
